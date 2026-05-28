import sqlite3

conn = sqlite3.connect("suelos.db")
cur = conn.cursor()

cur.execute("""
CREATE TABLE IF NOT EXISTS lecturas (
    id           INTEGER PRIMARY KEY AUTOINCREMENT,
    tipo_suelo   TEXT    NOT NULL,
    ph           REAL    NOT NULL,
    humedad_pct  REAL    NOT NULL,
    fertil       INTEGER NOT NULL DEFAULT 0,
    dato_cifrado TEXT    NOT NULL,
    hora_envio   TEXT    DEFAULT (datetime('now','localtime'))
)
""")

conn.commit()
conn.close()
print("Listo: base de datos creada correctamente.")