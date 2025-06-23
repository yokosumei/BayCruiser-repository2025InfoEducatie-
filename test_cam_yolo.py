
def detection_thread():
    global frame_buffer, yolo_output_frame, detected_flag, popup_sent, last_detection_time, frame_counter, event_location
    cam_x, cam_y = 320, 240
    PIXELS_PER_CM = 10
    object_present = False
    logging.info("Firul 2 (detectie) a pornit.")

    while True:
        if not streaming:
            time.sleep(0.1)
            continue

        frame_counter += 1
        if frame_counter % detection_frame_skip != 0:
             time.sleep(0.1)
            continue

        with lock:
            data = frame_buffer.copy() if frame_buffer else None
        if data is None:
            logging.warning("Nu există frame pentru detecție.") 
            time.sleep(0.05)
            continue

        frame = data["image"]
        gps_info = data["gps"]
        
        resized = cv2.resize(frame, (320, 240))  # doar pentru detecție
        results = model(resized, verbose=False)

        annotated = results[0].plot()

        # Adaugă text GPS și timestamp pe YOLO stream
        if gps_info["lat"] is not None and gps_info["lon"] is not None:
            gps_text = f"Lat: {gps_info['lat']:.6f} Lon: {gps_info['lon']:.6f} Alt: {gps_info['alt']:.1f}"
            timestamp_text = f"Timp: {time.strftime('%H:%M:%S', time.localtime(gps_info['timestamp']))}"
            cv2.putText(annotated, gps_text, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(annotated, timestamp_text, (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        names = results[0].names
        class_ids = results[0].boxes.cls.tolist()
        current_detection = False

        for i, cls_id in enumerate(class_ids):
            if names[int(cls_id)] == "om_la_inec":
                current_detection = True

                if not object_present:
                    detected_flag = True
                    popup_sent = True
                    last_detection_time = time.time()
                    object_present = True

                if gps_info["lat"] is not None:
                    event_location = LocationGlobalRelative(gps_info["lat"], gps_info["lon"], gps_info["alt"] or 10)

                box = results[0].boxes[i]
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                obj_x = (x1 + x2) // 2
                obj_y = (y1 + y2) // 2

                dx_cm = (obj_x - cam_x) / PIXELS_PER_CM
                dy_cm = (obj_y - cam_y) / PIXELS_PER_CM
                dist_cm = (dx_cm**2 + dy_cm**2)**0.5

                cv2.line(annotated, (cam_x, cam_y), (obj_x, obj_y), (0, 0, 255), 2)
                cv2.circle(annotated, (cam_x, cam_y), 5, (255, 0, 0), -1)
                cv2.circle(annotated, (obj_x, obj_y), 5, (0, 255, 0), -1)

                offset_text = f"x:{dx_cm:.1f}cm | y:{dy_cm:.1f}cm"
                dist_text = f"Dist: {dist_cm:.1f}cm"
                cv2.putText(annotated, offset_text, (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
                cv2.putText(annotated, dist_text, (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
                break

        if not current_detection:
            detected_flag = False
            popup_sent = False
            object_present = False

        with output_lock:
            yolo_output_frame = cv2.imencode('.jpg', annotated)[1].tobytes()
