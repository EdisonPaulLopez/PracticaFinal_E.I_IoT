import sqlite3
import base64
import json
from flask import Flask, request, jsonify, render_template
from Crypto.Cipher import AES
from Crypto.Util.Padding import unpad

app = Flask(__name__)

# ─── Misma clave que en la ESP32: "SueloIoT2024Key!" ───────
# En hex: 53 75 65 6C 6F 49 6F 54 32 30 32 34 4B 65 79 21
CLAVE_AES = bytes([
    0x53, 0x75, 0x65, 0x6C, 0x6F, 0x49, 0x6F, 0x54,
    0x32, 0x30, 0x32, 0x34, 0x4B, 0x65, 0x79, 0x21
])

# ─── IV fijo igual que en la ESP32: "IVSuelo012345678" ─────
IV_FIJO = b"IVSuelo012345678"


def descifrar(dato_b64):
    """
    Descifra el ciphertext en Base64 usando AES-128-CBC
    con el IV fijo que usa la ESP32.
    """
    ciphertext = base64.b64decode(dato_b64)
    cipher     = AES.new(CLAVE_AES, AES.MODE_CBC, IV_FIJO)
    descifrado = unpad(cipher.decrypt(ciphertext), AES.block_size)
    return json.loads(descifrado.decode("utf-8"))


def es_fertil(ph, humedad):
    """
    La ESP32 no envía este campo, así que lo calculamos aquí
    con la misma lógica que usa classifySoilComplete().
    """
    return 1 if (5.5 <= ph <= 7.5 and 55.0 <= humedad <= 80.0) else 0


# ── Ruta que recibe los datos de la ESP32 ──────────────────
API_KEY_VALIDA = "suelo2026"

@app.route("/api/soil", methods=["POST"])
def recibir_datos():
    body = request.get_json(force=True)

    # Verificar API key
    if body.get("api_key", "") != API_KEY_VALIDA:
        return jsonify({"error": "API key incorrecta"}), 401

    # Campos que envía la ESP32
    dato_b64 = body.get("data",    "")
    iv_texto = body.get("iv",      "IVSuelo012345678")  # por si acaso
    api_key  = body.get("api_key", "")

    if not dato_b64:
        return jsonify({"error": "No llegaron datos cifrados"}), 400

    # Descifrar
    try:
        datos = descifrar(dato_b64)
    except Exception as e:
        return jsonify({"error": f"No se pudo descifrar: {e}"}), 400

    # Extraer campos del JSON descifrado
    # (nombres en inglés tal como los manda la ESP32)
    tipo_suelo  = datos.get("soil_type",  "Desconocido")
    ph          = float(datos.get("ph",         0))
    humedad_pct = float(datos.get("humidity",   0))
    timestamp   = datos.get("timestamp", "")
    fertil      = es_fertil(ph, humedad_pct)

    # Guardar en la base de datos
    conn = sqlite3.connect("suelos.db")
    cur  = conn.cursor()
    cur.execute("""
        INSERT INTO lecturas (tipo_suelo, ph, humedad_pct, fertil, dato_cifrado)
        VALUES (?, ?, ?, ?, ?)
    """, (tipo_suelo, ph, humedad_pct, fertil, dato_b64))
    conn.commit()
    conn.close()

    print(f"[OK] Guardado: {tipo_suelo} | pH {ph} | Hum {humedad_pct}%")

    return jsonify({"ok": True, "guardado": tipo_suelo}), 201


# ── Devuelve todas las lecturas en JSON ────────────────────
@app.route("/lecturas")
def ver_lecturas():
    conn = sqlite3.connect("suelos.db")
    conn.row_factory = sqlite3.Row
    cur  = conn.cursor()
    cur.execute("SELECT * FROM lecturas ORDER BY hora_envio DESC")
    filas = [dict(f) for f in cur.fetchall()]
    conn.close()
    return jsonify(filas)


# ── Panel visual en el navegador ───────────────────────────
@app.route("/")
def panel():
    return render_template("index.html")


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True)