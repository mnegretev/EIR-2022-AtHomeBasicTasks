msg_speech = SoundRequest()
msg_speech.sound   = -3
msg_speech.command = 1
msg_speech.volume  = 1.0
#
# EJERCICIO
# Cambie la voz por alguna de las voces instaladas
#
msg_speech.arg2    = "voice_kal_diphone"
msg_speech.arg = text_to_say
