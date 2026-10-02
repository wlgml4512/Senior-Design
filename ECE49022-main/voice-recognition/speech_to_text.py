"""Azure speech-to-text session wrapper for ATLAS voice commands.

This file is relevant because it turns microphone audio into text that can be
parsed into robot commands, making it the spoken-language entry point for the
standalone voice subsystem.
"""

# Derived from code by Prof. Jeffrey Mark Siskind (qobi)
# Original source: https://github.com/qobi/ece49595nl/blob/main/speech_to_text_microsoft.py
# Modified for ATLAS Senior Design Project

import threading
import time

try:
    import azure.cognitiveservices.speech as speechsdk
except ModuleNotFoundError:
    speechsdk = None

from runtime_support import load_keys_module


_KEYS = load_keys_module()

class SpeechRecognizer:
    def __init__(self, silence_timeout=4.0, utterance_callback=None):
        if speechsdk is None:
            raise RuntimeError(
                "Azure Speech SDK is not installed. Install voice-recognition/requirements.txt."
            )
        if _KEYS is None or not getattr(_KEYS, "azure_key", None) or not getattr(_KEYS, "azure_region", None):
            raise RuntimeError(
                "Azure speech credentials were not found. Add azure_key and azure_region to repo-root keys.py."
            )

        self.listen = True
        self.speech_recognizer = None
        self.error_in_s2t_session = False
        self.utterance_fragments = []
        self.stop_speech_recognition = False
        self.done = False
        self.SILENCE_TIMEOUT = silence_timeout
        self.utterance_callback = utterance_callback
        self.beginning_of_silence = None
        self.speech_recognition_thread = None
        self._keys = _KEYS
        
    def handle_final_result(self, evt):
        response = "".join([i if ord(i)<128 else " " for i in evt.result.text])
        self.utterance_fragments.append(response)
    
    def handle_error(self, evt):
        self.error_in_s2t_session = True
    
    def handle_intermediate_result(self, evt):
        self.beginning_of_silence = time.time()
    
    def speech_recognition_thread_function(self):
        while not self.stop_speech_recognition:
            reconnect_time = 0.0
            while self.listen and not self.stop_speech_recognition:
                time.sleep(reconnect_time)
                try:
                    if self.speech_recognizer is None:
                        speech_config = speechsdk.SpeechConfig(
                            subscription=self._keys.azure_key,
                            region=self._keys.azure_region)
                        audio_config = speechsdk.audio.AudioConfig(
                            use_default_microphone=True)
                        self.speech_recognizer = speechsdk.SpeechRecognizer(
                            speech_config=speech_config, audio_config=audio_config)
                        self.speech_recognizer.recognized.connect(self.handle_final_result)
                        self.speech_recognizer.canceled.connect(self.handle_error)
                        self.speech_recognizer.recognizing.connect(
                            self.handle_intermediate_result)
                    
                    self.utterance_fragments = []
                    self.beginning_of_silence = time.time()
                    self.speech_recognizer.start_continuous_recognition()
                    print("DEBUG: Started listening...")
                    
                    while (self.listen and
                           not self.error_in_s2t_session and
                           not self.stop_speech_recognition):
                        now = time.time()
                        
                        # silence timeout handling
                        if now - self.beginning_of_silence > self.SILENCE_TIMEOUT:
                            print("DEBUG: Silence timeout reached.")
                            self.stop_speech_recognition = True
                            self.done = True
                            break
                        
                        if len(self.utterance_fragments) > 0:
                            sanitized_utterance = ""
                            for c in " ".join(self.utterance_fragments).strip():
                                if 0 <= ord(c) <= 127:
                                    sanitized_utterance += c
                            if sanitized_utterance == "":
                                time.sleep(0.1)
                                continue
                            self.utterance_fragments = []
                            
                            # Call callback if provided
                            if self.utterance_callback:
                                self.utterance_callback(sanitized_utterance)
                                
                        time.sleep(0.1)
                    
                    self.speech_recognizer.stop_continuous_recognition_async()
                    self.error_in_s2t_session = False
                except Exception as e:
                    print("Error in speech recognition session:", e)
                    reconnect_time += 0.1
                time.sleep(0.1)
            if not self.listen:
                time.sleep(0.1)
        self.stop_speech_recognition = False
    
    def start(self):
        self.speech_recognition_thread = threading.Thread(
            target=self.speech_recognition_thread_function)
        self.speech_recognition_thread.start()
    
    def stop(self):
        self.stop_speech_recognition = True
        if self.speech_recognition_thread:
            self.speech_recognition_thread.join()
    
    def is_done(self):
        return self.done

if __name__ == "__main__":
    def test_callback(text):
        print(f"Recognized text: {text}")
    
    recognizer = SpeechRecognizer(utterance_callback=test_callback)
    recognizer.start()
    
    try:
        while not recognizer.is_done():
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    
    recognizer.stop()


# EXAMPLE USAGE IN SEPERATE FILE:
# import stt_module
# import time

# def proccess_speech(text):
#     print(f"Recognized: {text}")
#     # add to user prompt or whatever you need to do

# recognizer = stt_module.SpeechRecognizer(
#     silence_timeout=2.0, utterance_callback=proccess_speech
# )
# recognizer.start()

# while not recognizer.is_done():
#     time.sleep(1)

# recognizer.stop()
