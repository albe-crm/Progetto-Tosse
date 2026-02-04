from flask import Flask, render_template, request, redirect, url_for, session, jsonify
from flask_sock import Sock
from datetime import datetime

app = Flask(__name__)
app.secret_key = "CHANGE_THIS_SECRET_KEY"
sock = Sock(app)

# ==============================
# LOGIN
# ==============================
VALID_USERNAME = "admin"
VALID_PASSWORD = "password"

# ==============================
# SERVER STATE
# ==============================
clients = set()

total_coughs = 0
hourly_coughs = [0] * 24
last_battery_voltage = None

# ==============================
# ROUTES
# ==============================
@app.route("/", methods=["GET", "POST"])
def login():
    if request.method == "POST":
        if request.form["username"] == VALID_USERNAME and request.form["password"] == VALID_PASSWORD:
            session["logged_in"] = True
            return redirect(url_for("patients"))
        return render_template("login.html", error="Invalid credentials")
    return render_template("login.html")


@app.route("/patients")
def patients():
    if not session.get("logged_in"):
        return redirect(url_for("login"))
    return render_template("patients.html")


@app.route("/dashboard")
def dashboard():
    if not session.get("logged_in"):
        return redirect(url_for("login"))
    return render_template("dashboard.html")


@app.route("/graphs")
def graphs():
    if not session.get("logged_in"):
        return redirect(url_for("login"))
    return render_template("graphs.html")


@app.route("/api/state")
def api_state():
    return jsonify(
        total_coughs=total_coughs,
        hourly_coughs=hourly_coughs,
        battery_voltage=last_battery_voltage
    )

@app.route("/api/reset", methods=["POST"])
def api_reset():
    global total_coughs, hourly_coughs

    if not session.get("logged_in"):
        return jsonify({"error": "unauthorized"}), 401

    total_coughs = 0
    hourly_coughs = [0] * 24

    return jsonify({"status": "ok"})

@app.route("/logout")
def logout():
    session.clear()
    return redirect(url_for("login"))

# ==============================
# WEBSOCKET (MCU + BROWSERS)
# ==============================
@sock.route("/ws")
def websocket(ws):
    global total_coughs, hourly_coughs, last_battery_voltage

    print("WS client connected")
    clients.add(ws)

    try:
        while True:
            data = ws.receive()
            if data is None:
                continue

            print("RECEIVED:", data)   # <<< DEBUG CONFIRMATION

            parts = data.split(',')
            if len(parts) == 5:
                try:
                    msg_type = int(parts[0])
                    battery = float(parts[4])

                    if msg_type == 0:
                        total_coughs += 1
                        hourly_coughs[datetime.now().hour] += 1

                    last_battery_voltage = battery

                except ValueError:
                    pass

            for client in list(clients):
                try:
                    client.send(data)
                except Exception:
                    clients.discard(client)

    finally:
        clients.discard(ws)
        print("WS client disconnected")

# ==============================
# MAIN (CRITICAL SETTINGS)
# ==============================
if __name__ == "__main__":
    app.run(
        host="0.0.0.0",
        port=5000,
        debug=False,          # ❌ MUST BE FALSE
        use_reloader=False    # ❌ MUST BE FALSE
    )
