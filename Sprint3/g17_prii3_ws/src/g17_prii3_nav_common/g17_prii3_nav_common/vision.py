"""Detección de Sprint 2 reducida a IDs y centros; sin calibración ajena."""
import cv2


def detector_aruco():
    cv2.setNumThreads(2)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    if hasattr(cv2.aruco, 'ArucoDetector'):
        return cv2.aruco.ArucoDetector(dictionary).detectMarkers
    params = cv2.aruco.DetectorParameters_create()
    return lambda image: cv2.aruco.detectMarkers(image, dictionary, parameters=params)


def detect(image, detector):
    corners, ids, _ = detector(image)
    markers = []
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(image, corners, ids)
        for border, marker_id in zip(corners, ids.flatten()):
            x, y = border.reshape(4, 2).mean(axis=0)
            markers.append(dict(id=int(marker_id), x=round(float(x), 1), y=round(float(y), 1)))
    return sorted(markers, key=lambda marker: marker['id'])
