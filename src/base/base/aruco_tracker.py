"""OpenCV ArUco 2D detector used by the Raspberry Pi camera node.

This module deliberately reports image measurements only.  Camera calibration
and pose estimation are separate commissioning steps; pixel values must not be
treated as metres, millimetres, or a robot yaw angle.
"""
import math

import cv2
# 영상 노드가 여러 개 동시에 돌아 4코어에 스레드가 몰린다. 프로세스
# 단위로 이미 병렬이므로 OpenCV 내부 스레드는 1로 묶는 편이 총
# 처리량이 낫다(2026-09-01 부하 실측).
cv2.setNumThreads(1)
import numpy as np


DICTIONARIES = {
    'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
}


class ArucoMarkerTracker:
    """Detect allowed DICT_4X4_50 markers and draw a diagnostic overlay."""

    def __init__(self, dictionary_name='DICT_4X4_50', allowed_ids=(1, 2, 3),
                 target_id=-1, marker_size_mm=40.0, plate_size_mm=45.0,
                 border_bits=1, min_side_px=20.0, jpeg_quality=82,
                 camera_matrix=None, dist_coeffs=None,
                 calibration_width=0, calibration_height=0,
                 stream_width=0, stream_height=0,
                 draw_axes=False, axes_length_mm=30.0,
                 clahe_fallback=True, equalize_fallback=True,
                 equalize_primary=False):
        if dictionary_name not in DICTIONARIES:
            raise ValueError(
                f'Unsupported ArUco dictionary {dictionary_name!r}; '
                f'available={sorted(DICTIONARIES)}')
        self.dictionary_name = dictionary_name
        self.allowed_ids = tuple(sorted({int(value) for value in allowed_ids}))
        self.target_id = int(target_id)
        self.marker_size_mm = float(marker_size_mm)
        self.plate_size_mm = float(plate_size_mm)
        self.border_bits = int(border_bits)
        self.min_side_px = float(min_side_px)
        self.jpeg_quality = int(jpeg_quality)
        if not self.allowed_ids:
            raise ValueError('allowed_ids must not be empty')
        if min(self.marker_size_mm, self.plate_size_mm, self.min_side_px) <= 0.0:
            raise ValueError('marker/plate/min-side values must be positive')
        if self.border_bits != 1:
            raise ValueError('This project fixes ArUco borderBits to 1')

        # Pose (x/y/z + orientation) is only computed when a camera
        # calibration is configured, and only while the live frame size
        # matches the size that calibration was done at -- camera_matrix's
        # fx/fy/cx/cy are pixel values tied to a specific resolution.
        self.camera_matrix = (
            np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
            if camera_matrix is not None else None)
        self.dist_coeffs = (
            np.asarray(dist_coeffs, dtype=np.float64).reshape(1, -1)
            if dist_coeffs is not None else None)
        self.calibration_width = int(calibration_width)
        self.calibration_height = int(calibration_height)
        # 검출/pose는 항상 원본 해상도 프레임에서 계산한다 -- 이 크기는
        # vision/image/compressed로 내보내는 주석 프레임만 줄인다(웹 스트림
        # 지연 완화 목적). 0이면 원본 해상도 그대로 인코드한다.
        self.stream_width = int(stream_width)
        self.stream_height = int(stream_height)
        self._stream_size = (
            (self.stream_width, self.stream_height)
            if self.stream_width > 0 and self.stream_height > 0 else None)
        # 선택된(target) 마커 위에 solvePnP 결과로 3축(X/Y/Z) 좌표계를
        # 그릴지 여부 -- marker_vision.py가 marker/axes_enable(Bool)로
        # 실행 중에도 이 값을 바꿀 수 있게 평범한 속성으로 둔다. 기본은
        # 꺼짐(실시간성 우선), 필요할 때만 웹 콘솔에서 켠다.
        self.draw_axes = bool(draw_axes)
        self.axes_length_mm = float(axes_length_mm)
        self.clahe_fallback = bool(clahe_fallback)
        # CLAHE(국소 대비)로도 안 잡히는 프레임이 있다. 2026-09-01 실측:
        # 골판지 상자에 붙인 ID 1 마커가 기본/CLAHE 로는 후보 22개를 전부
        # 버리는데(디코딩 0건), 전역 equalizeHist 를 걸면 바로 잡혔다.
        # 전체 화면 밝기가 좁은 대역에 몰릴 때 국소 보정만으로는 검은
        # 테두리와 흰 칸의 분리가 부족해서다.
        self.equalize_fallback = bool(equalize_fallback)
        # 조명이 나쁜 현장에서는 기본 경로가 사실상 항상 실패해서, 폴백이
        # 예외가 아니라 상시 경로가 된다. 그러면 프레임마다 검출을 세 번
        # 돌게 되어 CPU 를 3배 쓴다(2026-09-01 실측: marker_vision 74%,
        # 카메라 18fps, 검출률 0%). 그럴 때는 평활화를 1차로 올려 한 번만
        # 돌린다. 조명이 좋아지면 다시 false 로 되돌리는 게 더 빠르다.
        self.equalize_primary = bool(equalize_primary)
        self._clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        self.pose_available = (
            self.camera_matrix is not None and self.dist_coeffs is not None)
        # OpenCV's own object-point convention for a single square marker:
        # TL, TR, BR, BL in the marker's own plane (Z out of the marker),
        # matching the corner order detectMarkers() returns.
        half = self.marker_size_mm / 2.0
        self._object_points = np.array([
            [-half, half, 0.0], [half, half, 0.0],
            [half, -half, 0.0], [-half, -half, 0.0],
        ], dtype=np.float64)

        dictionary_id = DICTIONARIES[dictionary_name]
        self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        self.parameters = self._parameters()
        self.detector = (
            cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
            if hasattr(cv2.aruco, 'ArucoDetector') else None)

    @staticmethod
    def _parameters():
        # OpenCV 4.6 on this Pi exposes both names, but only the factory is the
        # stable API for the legacy detectMarkers path.
        if hasattr(cv2.aruco, 'DetectorParameters_create'):
            parameters = cv2.aruco.DetectorParameters_create()
        else:
            parameters = cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        parameters.minMarkerPerimeterRate = 0.02
        return parameters

    @staticmethod
    def _decode(jpeg):
        return cv2.imdecode(
            np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)

    def _detect(self, gray):
        if self.detector is not None:
            return self.detector.detectMarkers(gray)
        return cv2.aruco.detectMarkers(
            gray, self.dictionary, parameters=self.parameters)

    def _pose(self, corners, frame_width, frame_height):
        """solvePnP로 x/y/z(mm, 카메라 좌표계)와 roll/pitch/yaw(deg)를 구한다.

        camera_matrix가 없거나(캘리브레이션 전) 현재 프레임 해상도가
        캘리브레이션 당시와 다르면 None을 돌려준다 — 안 맞는 해상도로
        계산하면 조용히 틀린 값이 나오기 때문에 아예 계산하지 않는다.
        """
        if not self.pose_available:
            return None
        if (self.calibration_width and frame_width != self.calibration_width) or \
                (self.calibration_height and frame_height != self.calibration_height):
            return None
        image_points = np.asarray(corners, dtype=np.float64).reshape(4, 1, 2)
        ok, rvec, tvec = cv2.solvePnP(
            self._object_points, image_points,
            self.camera_matrix, self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return None
        rotation, _ = cv2.Rodrigues(rvec)
        # Tait-Bryan roll(X)/pitch(Y)/yaw(Z), extracted from the rotation
        # matrix in the usual XYZ-extrinsic convention.
        roll = math.degrees(math.atan2(rotation[2, 1], rotation[2, 2]))
        pitch = math.degrees(math.atan2(
            -rotation[2, 0],
            math.hypot(rotation[2, 1], rotation[2, 2])))
        yaw = math.degrees(math.atan2(rotation[1, 0], rotation[0, 0]))
        return {
            'x_mm': float(tvec[0, 0]), 'y_mm': float(tvec[1, 0]),
            'z_mm': float(tvec[2, 0]),
            'roll_deg': roll, 'pitch_deg': pitch, 'yaw_deg': yaw,
            # 축각(axis-angle) 회전 원값 — Euler 대신 이걸 쓰고 싶을 때용.
            'rvec': [float(v) for v in rvec.reshape(-1)],
        }

    @staticmethod
    def _measurement(marker_id, corners, frame_width, frame_height):
        points = np.asarray(corners, dtype=np.float32).reshape(4, 2)
        center = points.mean(axis=0)
        edges = np.roll(points, -1, axis=0) - points
        side_lengths = np.linalg.norm(edges, axis=1)
        area = abs(float(cv2.contourArea(points)))
        rotation_deg = math.degrees(math.atan2(
            float(edges[0, 1]), float(edges[0, 0])))
        return {
            'id': int(marker_id),
            'robot': int(marker_id),
            'center_x_px': float(center[0]),
            'center_y_px': float(center[1]),
            'offset_x_px': float(center[0] - frame_width / 2.0),
            'offset_y_px': float(center[1] - frame_height / 2.0),
            'side_px': float(side_lengths.mean()),
            'area_px2': area,
            # This is rotation in the image plane, not calibrated 3D yaw.
            'image_rotation_deg': float(rotation_deg),
            'corners_px': [[float(x), float(y)] for x, y in points],
        }

    def process(self, jpeg):
        frame = self._decode(jpeg)
        if frame is None:
            raise ValueError('Camera JPEG could not be decoded')
        frame_height, frame_width = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.equalize_primary:
            gray = cv2.equalizeHist(gray)
        corners, ids, rejected = self._detect(gray)
        # Uneven indoor lighting can make the black border disappear from one
        # thresholded frame. Retry only failed frames with local contrast
        # equalisation so the normal fast path is unchanged.
        if ids is None and self.clahe_fallback:
            enhanced = self._clahe.apply(gray)
            corners, ids, rejected = self._detect(enhanced)
        # 국소 보정이 실패한 프레임만 전역 평활화로 마지막 한 번 더 본다.
        # 정상 프레임의 빠른 경로는 그대로 두기 위해 순서를 이렇게 둔다.
        if ids is None and self.equalize_fallback:
            corners, ids, rejected = self._detect(cv2.equalizeHist(gray))

        decoded_ids = [] if ids is None else [int(v) for v in ids.reshape(-1)]
        allowed, rejected_ids = [], []
        for marker_id, marker_corners in zip(decoded_ids, corners):
            measurement = self._measurement(
                marker_id, marker_corners, frame_width, frame_height)
            if marker_id not in self.allowed_ids or measurement['side_px'] < self.min_side_px:
                rejected_ids.append(marker_id)
                colour = (60, 60, 220)
            else:
                pose = self._pose(marker_corners, frame_width, frame_height)
                if pose is not None:
                    measurement.update(pose)
                allowed.append(measurement)
                colour = (40, 220, 40)
            points = np.asarray(marker_corners, dtype=np.int32).reshape(4, 2)
            cv2.polylines(frame, [points], True, colour, 3)
            for index, point in enumerate(points):
                cv2.circle(frame, tuple(point), 5, (255, 255, 0), -1)
                cv2.putText(frame, str(index), tuple(point + (7, -7)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            center = tuple(np.round(points.mean(axis=0)).astype(int))
            cv2.drawMarker(frame, center, colour, cv2.MARKER_CROSS, 22, 2)
            cv2.putText(frame, f'ID {marker_id} | Robot {marker_id}',
                        (int(points[:, 0].min()), max(24, int(points[:, 1].min()) - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, colour, 2)

        # Prefer the configured target.  With target_id=-1 the largest allowed
        # marker is selected, while every decoded marker remains in `markers`.
        allowed.sort(key=lambda item: item['area_px2'], reverse=True)
        selected = next(
            (item for item in allowed if item['id'] == self.target_id),
            allowed[0] if allowed and self.target_id < 0 else None)

        # selected에 'rvec'가 있다는 것 자체가 이미 pose_available과 해상도
        # 일치를 통과했다는 뜻이므로(위 _pose() 참고) 여기서 다시 확인하지
        # 않는다. 원본 해상도 프레임에 그려서 stream_width/height 리사이즈가
        # 있어도 좌표가 그대로 따라가게 한다(아래에서 프레임 전체를 줄이므로).
        if self.draw_axes and selected is not None and 'rvec' in selected:
            rvec = np.asarray(selected['rvec'], dtype=np.float64).reshape(3, 1)
            tvec = np.asarray(
                [selected['x_mm'], selected['y_mm'], selected['z_mm']],
                dtype=np.float64).reshape(3, 1)
            cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs,
                               rvec, tvec, self.axes_length_mm, 3)

        label = (
            f"ARUCO {self.dictionary_name} | IDs "
            f"{','.join(str(item['id']) for item in allowed) or '-'}")
        cv2.rectangle(frame, (0, 0), (frame_width, 34), (0, 0, 0), -1)
        cv2.putText(frame, label, (10, 24), cv2.FONT_HERSHEY_SIMPLEX,
                    0.58, (255, 255, 255), 2)
        # 검출/pose 계산은 이미 끝났으므로(위) 여기서부터는 표시용 프레임만
        # 다룬다 -- stream_width/height가 설정돼 있으면 여기서만 줄인다.
        display_frame = (
            cv2.resize(frame, self._stream_size, interpolation=cv2.INTER_AREA)
            if self._stream_size is not None else frame)
        encoded, annotated = cv2.imencode(
            '.jpg', display_frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        if not encoded:
            raise ValueError('Annotated ArUco JPEG could not be encoded')

        pose_available = self.pose_available and not (
            (self.calibration_width and frame_width != self.calibration_width) or
            (self.calibration_height and frame_height != self.calibration_height))
        result = {
            'detected': selected is not None,
            'target_detected': selected is not None,
            'detected_ids': sorted({item['id'] for item in allowed}),
            'decoded_ids': sorted(set(decoded_ids)),
            'rejected_ids': sorted(set(rejected_ids)),
            'rejected_candidates': len(rejected),
            'markers': allowed,
            'frame_width': int(frame_width),
            'frame_height': int(frame_height),
            'pose_available': bool(pose_available),
            '_jpeg': bytes(annotated),
        }
        if selected is not None:
            result.update(selected)
        return result
