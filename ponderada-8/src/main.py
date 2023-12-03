import os
from pathlib import Path
from playsound import playsound
import openai 

openai.api_key=""

print(openai.api_key)


def audio_to_text(audio_path):
    print("Transcribing audio...")
    audio_file= open(audio_path, "rb")
    transcript = openai.audio.transcriptions.create(
        model="whisper-1", 
        file=audio_file,
        response_format="text"
    )
    print(transcript)
    return transcript

def translate_text(text, source_language, target_language):
    print("Translating text...")
    response = openai.chat.completions.create(
        model="gpt-3.5-turbo",
        messages=[
            {"role": "system", "content": "Translete from " + source_language + " to " + target_language + "."},
            
            {"role": "user", "content": text}
        ]
    )
    print(response.choices[0].message.content)
    return response.choices[0].message.content

def text_to_audio(text):
    print("Generating audio...")
    speech_file_path = Path(__file__).parent / "media/speech.mp3"
    response = openai.audio.speech.create(
        model="tts-1",
        voice="alloy",
        input=text,
    )
    response.stream_to_file(speech_file_path)
    return speech_file_path

def play_audio(audio_path):
   print("Playing audio...")
   playsound(str(audio_path).replace('/', '\\'))

def main():
    audio_file_path = "./media/test.mp3"
    text = audio_to_text(audio_file_path)
    translated_text = translate_text(text, "pt", "en")
    audio_path = text_to_audio(translated_text)
    play_audio(audio_path)

if __name__ == "__main__":
    main()

