from flask import Response, jsonify

@app.route("/video_feed")
def video_feed():
    def generate():
        logging.info("[FLASK] Client conectat la /video_feed")
        while True:
            if not streaming:
                logging.debug("[FLASK] Streaming oprit — se așteaptă pornirea streamului.")
                time.sleep(0.1)
                continue
            with output_lock:
                frame = output_frame if output_frame else blank_frame()
            try:
                yield (b"--frame
"
                       b"Content-Type: image/jpeg

" + frame + b"
")
                logging.debug("[FLASK] Frame trimis către client /video_feed")
            except Exception as e:
                logging.error(f"[FLASK] Eroare în fluxul video_feed: {e}")
            time.sleep(0.05)
    return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/yolo_feed")
def yolo_feed():
    def generate():
        logging.info("[FLASK] Client conectat la /yolo_feed")
        while True:
            if not streaming:
                logging.debug("[FLASK] Streaming oprit — se așteaptă pornirea streamului YOLO.")
                time.sleep(0.1)
                continue
            with output_lock:
                frame = yolo_output_frame if yolo_output_frame else blank_frame()
            try:
                yield (b"--frame
"
                       b"Content-Type: image/jpeg

" + frame + b"
")
                logging.debug("[FLASK] Frame trimis către client /yolo_feed")
            except Exception as e:
                logging.error(f"[FLASK] Eroare în fluxul yolo_feed: {e}")
            time.sleep(0.05)
    return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/start_stream")
def start_stream():
    global streaming
    logging.info("[FLASK] /start_stream apelat")
    streaming = True
    return jsonify({"status": "started"})

@app.route("/stop_stream")
def stop_stream():
    global streaming
    logging.info("[FLASK] /stop_stream apelat")
    streaming = False
    return jsonify({"status": "stopped"})

@app.route("/detection_status")
def detection_status():
    logging.debug("[FLASK] /detection_status apelat")
    return jsonify({"detected": popup_sent})

@app.route("/misca")
def activate():
    logging.info("[FLASK] /misca apelat — se activează servomotorul")
    activate_servos()
    return "Servomotor activat"

@app.route("/return_to_event")
def return_to_event():
    global event_location
    logging.info("[FLASK] /return_to_event apelat")
    if not event_location:
        logging.warning("[FLASK] Nu există coordonate salvate pentru revenirea dronei.")
        return jsonify({"status": "no event location"})

    try:
        logging.info(f"[FLASK] Se trimite drona înapoi la: {event_location}")
        gps_provider.vehicle.simple_goto(event_location)
        return jsonify({
            "status": "returning",
            "lat": event_location.lat,
            "lon": event_location.lon,
            "alt": event_location.alt
        })
    except Exception as e:
        logging.error(f"[FLASK] Eroare la trimiterea dronei către locație: {e}")
        return jsonify({"status": "error", "message": str(e)})
