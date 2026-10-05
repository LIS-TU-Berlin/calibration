import cv2


class ArucoDetector:
    def __init__(self, detect_board=False, verbose=1):
        self.detect_board = detect_board
        self.verbose = verbose

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
        self.board = cv2.aruco.CharucoBoard((5, 7), 0.055, 0.041, self.aruco_dict)

        detector_params = cv2.aruco.DetectorParameters()
        detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, detector_params)
        self.charuco_detector = cv2.aruco.CharucoDetector(self.board, detectorParams=detector_params)

        if self.verbose > 0:
            cv2.namedWindow("preview", cv2.WINDOW_NORMAL)
            cv2.moveWindow("preview", 100, 100)

    def step(self, image, draw_markers=False):
        preview = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        gray = cv2.cvtColor(preview, cv2.COLOR_RGB2GRAY)

        if not self.detect_board:
            marker_corners, marker_ids, rejected = self.aruco_detector.detectMarkers(gray)
            if draw_markers and marker_ids is not None:
                cv2.aruco.drawDetectedMarkers(preview, marker_corners, marker_ids)
        else:
            charuco_corners, charuco_ids, marker_corners, marker_ids = self.charuco_detector.det(gray)
            if draw_markers and charuco_ids is not None:
                cv2.aruco.drawDetectedCornersCharuco(preview, charuco_corners, charuco_ids)

        if self.verbose > 0:
            # cv2.putText(preview, f"saved: {saved}/{args.count}", (50, 120), cv2.FONT_HERSHEY_SIMPLEX, 5.0, (0, 255, 0), 5)
            cv2.imshow("preview", preview)
            cv2.resizeWindow("preview", 1000, int(1000 * preview.shape[0] / preview.shape[1]))
            cv2.waitKey(1)

        # should return something, if not used for display only..
