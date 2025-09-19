import whisper
from sklearn.feature_extraction.text import TfidfVectorizer
from sklearn.naive_bayes import MultinomialNB

phishing_keywords = [
    "password", "otp", "atm", "pin", "account", "verify", "blocked", "suspended", "urgent", "immediately",
    "bank", "credit", "lottery", "winner", "tax", "refund", "expired", "security", "confirm", "deposit"
]


model = whisper.load_model('base') #loading basic whisper model

#  Transcribe audio file to text
def transcribe_audio(file_path):
    result = model.transcribe(file_path)
    return result['text'].lower().strip()

#  Simple keyword check 
def has_phishing_keyword(transcribed_text):
    for kw in phishing_keywords:
        if kw in transcribed_text:
            return kw  # Return the exact matched keyword
    return None

#  Main function for complete pipeline
def detect_phishing(audio_file):
    transcribed_command = transcribe_audio(audio_file)
    print(f"Transcribed Command: \"{transcribed_command}\"")

    matched_keyword = has_phishing_keyword(transcribed_command)
    if matched_keyword:
        print(f"Phishing Detected! Keyword: \"{matched_keyword}\"")
        # Trigger vibration or other alert here
    else:
        print("No phishing detected.")
 
# Run it on an example audio file
audio_path = "example_audio.wav"
detect_phishing(audio_path)
