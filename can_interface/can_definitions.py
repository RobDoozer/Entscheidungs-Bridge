# can_definitions.py

# Dummy-ID zur CAN-Analyse (kann später entfernt werden)
ID_FAHRT = 0x101

# Hindernisse: keine festen Bits mehr notwendig, da wir alles über Bitfelder aus ROS-Topics /bool_topic_X regeln
# Falls du andere IDs beobachten willst, kannst du sie hier optional dokumentieren (nur für CAN-Logging)

# Bitmaske Beispiel zur Lesbarkeit (optional)
HINDERNIS_NONE = 0x00  # keine Bits gesetzt

# Richtungscodes für ROS-Fahrbefehl (UInt8)
FAHREN_STOPP = 0
FAHREN_VORWAERTS = 1
FAHREN_RUECKWAERTS = 2
