import struct

def parse_vlq(data, pos):
    val = 0
    while True:
        b = data[pos]; pos += 1
        val = (val << 7) | (b & 0x7F)
        if not (b & 0x80): break
    return val, pos

with open("c_major_scale.mid", "rb") as f:
    data = f.read()

fmt, tracks, ppq = struct.unpack_from(">HHH", data, 8)
print("Format:", fmt, " Tracks:", tracks, " PPQ:", ppq)

pos = 14
pos += 8
tempo = 500000
tick = 0

note_names = ['C','C#','D','D#','E','F','F#','G','G#','A','A#','B']

while pos < len(data):
    delta, pos = parse_vlq(data, pos)
    tick += delta
    status = data[pos]; pos += 1

    if status == 0xFF:
        meta = data[pos]; pos += 1
        mlen, pos = parse_vlq(data, pos)
        if meta == 0x51:
            tempo = int.from_bytes(data[pos:pos+3], 'big')
            bpm = 60000000 // tempo
            print("Tempo:", tempo, "us/beat =", bpm, "BPM")
        pos += mlen
    elif (status & 0xF0) == 0x90:
        note, vel = data[pos], data[pos+1]; pos += 2
        ms = (tick * tempo) // (ppq * 1000)
        name = note_names[note % 12] + str((note // 12) - 1)
        if vel > 0:
            print("  tick", tick, " ms", ms, "  NoteOn ", name, " MIDI", note, " vel", vel)
    elif (status & 0xF0) == 0x80:
        pos += 2
