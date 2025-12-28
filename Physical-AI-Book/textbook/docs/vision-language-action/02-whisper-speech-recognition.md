---
title: "Chapter 2: Whisper Speech Recognition"
slug: /vision-language-action/whisper-speech-recognition
sidebar_label: "2. Whisper Speech Recognition"
sidebar_position: 3
toc: true
description: "Integrate OpenAI Whisper for real-time, noise-robust, multilingual speech recognition in ROS 2 humanoid robotics systems."
---

# Chapter 2: Whisper Speech Recognition

## Introduction

Speech recognition is the first critical layer in Voice-to-Action pipelines, transforming audio waveforms into text that LLMs can process. Traditional automatic speech recognition (ASR) systems like Google Speech API or Amazon Transcribe work well in quiet environments but degrade rapidly with background noise, accents, or domain-specific vocabulary. For humanoid robots operating in homes, hospitals, or warehouses, these conditions are the norm, not the exception.

**OpenAI Whisper** (released September 2022) represents a paradigm shift: a 680-million-parameter transformer model trained on 680,000 hours of multilingual audio, achieving human-level accuracy even in noisy, diverse acoustic environments. Whisper supports 99+ languages, handles code-switching (mixing languages mid-sentence), and is Apache 2.0 licensed (free for commercial use). For robotics, Whisper's noise robustness and zero-shot generalization make it the default choice for voice interfaces.

This chapter covers Whisper's architecture, real-time vs. batch transcription trade-offs, noise robustness techniques, multilingual support, and ROS 2 integration via `audio_common` and custom nodes. By the end, you'll understand how to deploy Whisper on robot hardware (Nvidia Jetson, x86 servers) and achieve &lt;500ms latency for interactive voice control.

## Why Whisper for Robotics?

Humanoid robots face unique speech recognition challenges compared to static devices (Alexa, Google Home):

### 1. **Noisy Environments**
- **Challenge**: Robots operate near motors, conveyor belts, HVAC systems, and human chatter.
- **Whisper Advantage**: Trained on diverse noise conditions (street noise, music, overlapping speakers). Achieves 95% accuracy at 10 dB SNR (signal-to-noise ratio), vs. 60% for traditional ASR.
- **Example**: Warehouse robot near forklift beeping (85 dB background) still transcribes "Bring pallet to loading dock."

### 2. **Multilingual Support**
- **Challenge**: Robots deployed globally must understand local languages (Spanish in Mexico, Mandarin in China, Hindi in India).
- **Whisper Advantage**: Single model supports 99 languages, trained on native speaker data. No need to deploy separate models per language.
- **Example**: Code-switching: "Robot, go to the cocina and traeme a glass of water" (Spanish + English).

### 3. **Domain-Specific Vocabulary**
- **Challenge**: Medical terms ("administer saline IV"), warehouse jargon ("pallet jack, SKU"), technical commands ("execute ROS action server").
- **Whisper Advantage**: Zero-shot generalization from pretraining on diverse internet data (Wikipedia, podcasts, audiobooks).
- **Example**: "Navigate to ASRS" (Automated Storage and Retrieval System) → correctly transcribed without fine-tuning.

### 4. **No Cloud Dependency**
- **Challenge**: Cloud APIs (Google Speech, AWS Transcribe) require internet, introduce latency (100–300ms), and raise privacy concerns.
- **Whisper Advantage**: Runs locally on Nvidia GPUs (Jetson Orin: 100 TOPS, can run Whisper Large in real-time).
- **Example**: Robot in hospital must process patient data locally (HIPAA compliance).

### 5. **Timestamps and Alignment**
- **Challenge**: Robotics needs word-level timestamps for synchronization (e.g., highlight "cup" when robot looks at cup).
- **Whisper Advantage**: Outputs word-level timestamps and confidence scores for each segment.
- **Example**: "Go [0.0s] to [0.5s] the [0.7s] kitchen [1.0s] and [1.5s] bring [1.8s] me [2.0s] a [2.2s] cup [2.5s]."

## Whisper Architecture

Whisper is an **encoder-decoder transformer** model, trained on 680,000 hours of supervised data from the web. Unlike unsupervised models (Wav2Vec 2.0), Whisper is trained end-to-end on (audio, text) pairs, making it more robust to diverse accents and noise.

### Model Sizes

| Model      | Parameters | Relative Speed | English-Only WER | Multilingual WER | Use Case               |
|------------|------------|----------------|------------------|------------------|------------------------|
| Tiny       | 39M        | 32×            | 5.0%             | 12.3%            | Edge devices (Raspberry Pi) |
| Base       | 74M        | 16×            | 4.3%             | 10.1%            | Jetson Nano            |
| Small      | 244M       | 6×             | 3.9%             | 8.4%             | Jetson Xavier          |
| Medium     | 769M       | 2×             | 3.5%             | 7.2%             | Jetson Orin            |
| Large      | 1550M      | 1×             | 2.9%             | 6.1%             | Server (RTX 3090)      |
| Large-v3   | 1550M      | 1×             | 2.3%             | 5.5%             | Latest (2023), best accuracy |

**WER** = Word Error Rate (lower is better; human-level ≈ 5%)

**Recommendation for Humanoid Robotics**:
- **Mobile robots** (battery-powered, Jetson Orin): Whisper **Medium** (769M params, 200ms latency on Orin)
- **Server-based** (tethered, RTX 4090): Whisper **Large-v3** (2.3% WER, 100ms latency)

### Encoder-Decoder Architecture

```
Audio Waveform (16 kHz)
        ↓
[Log-Mel Spectrogram] (80 channels, 3000 time steps for 30s audio)
        ↓
┌────────────────────────────────────────┐
│          ENCODER (12-24 layers)        │
│  - Convolution layers (downsample 2x)  │
│  - Transformer blocks (self-attention) │
│  - Output: 1500 context vectors        │
└───────────────┬────────────────────────┘
                │ (Encoded audio context)
                ↓
┌────────────────────────────────────────┐
│         DECODER (12-24 layers)         │
│  - Cross-attention to encoder output   │
│  - Causal self-attention (left-to-right)│
│  - Predicts text tokens autoregressively│
└────────────────────────────────────────┘
        ↓
[Output Text] "Go to the kitchen and bring me a cup"
```

**Key Innovations**:
1. **Multilingual Training**: Single model trained on 99 languages (vs. separate models per language in traditional ASR).
2. **Task Conditioning**: Decoder takes special tokens to specify task (transcribe, translate, detect language).
3. **Timestamps**: Decoder predicts `<|timestamp|>` tokens alongside text (e.g., `<|0.0|> Go <|0.5|> to <|0.7|> the <|1.0|> kitchen`).

## Real-Time vs. Batch Transcription

Whisper was designed for **batch processing** (transcribe entire audio file). For robotics, we need **real-time streaming** (transcribe as user speaks). Two approaches:

### 1. Batch Mode (Non-Streaming)

**How it works**:
- Buffer audio until user stops speaking (voice activity detection, VAD)
- Pass entire utterance to Whisper (e.g., 3 seconds of audio)
- Get full transcription after processing

**Advantages**:
- Higher accuracy (model sees full context)
- Simple implementation (no sliding windows)

**Disadvantages**:
- Latency = silence detection (0.5s) + inference (0.2s) = 0.7s minimum
- User must pause between commands

**ROS 2 Implementation**:
```python
import whisper
import numpy as np
from audio_common_msgs.msg import AudioStamped
from std_msgs.msg import String

class WhisperBatchNode(Node):
    def __init__(self):
        super().__init__('whisper_batch')
        self.model = whisper.load_model("medium")  # 769M params
        self.audio_buffer = []
        self.is_speaking = False

        self.sub = self.create_subscription(
            AudioStamped, '/audio_input', self.audio_callback, 10)
        self.pub = self.create_publisher(String, '/transcription', 10)

    def audio_callback(self, msg):
        audio_chunk = np.frombuffer(msg.audio.data, dtype=np.int16)

        # Voice Activity Detection (VAD)
        energy = np.sqrt(np.mean(audio_chunk ** 2))
        if energy > 500:  # Threshold for speech (tune empirically)
            self.is_speaking = True
            self.audio_buffer.extend(audio_chunk)
        elif self.is_speaking:
            # Silence detected after speech → transcribe
            self.transcribe_buffer()
            self.audio_buffer = []
            self.is_speaking = False

    def transcribe_buffer(self):
        if len(self.audio_buffer) < 8000:  # Ignore &lt;0.5s utterances
            return

        audio_array = np.array(self.audio_buffer, dtype=np.float32) / 32768.0
        result = self.model.transcribe(audio_array, language="en")

        msg = String()
        msg.data = result["text"]
        self.pub.publish(msg)
        self.get_logger().info(f"Transcription: {result['text']}")
```

**Latency Breakdown** (Whisper Medium on Jetson Orin):
- VAD silence detection: 500ms
- Whisper inference: 200ms (3s audio)
- Total: **700ms**

### 2. Streaming Mode (Sliding Window)

**How it works**:
- Process audio in overlapping 2-second windows (e.g., [0–2s], [1.5–3.5s], [3–5s])
- Merge overlapping transcriptions

**Advantages**:
- Lower latency (partial results every 0.5s)
- Better for conversational interactions

**Disadvantages**:
- Risk of word-splitting at window boundaries ("kit-chen" → "kit" + "chen")
- More complex implementation (requires merging logic)

**Streaming Implementation** (Pseudocode):
```python
def streaming_transcribe(audio_stream):
    window_size = 2.0  # seconds
    hop_size = 0.5     # seconds (overlap 1.5s)

    for window in sliding_windows(audio_stream, window_size, hop_size):
        partial_result = whisper.transcribe(window)

        # Merge with previous results (deduplicate overlapping words)
        merged_text = merge_transcriptions(previous_text, partial_result)
        yield merged_text  # Stream partial transcription
```

**Trade-Off**: Streaming reduces latency but increases complexity. For command-based robotics ("Go to kitchen"), batch mode is simpler and sufficient. For conversational robots ("Tell me about your day"), streaming is better.

## Noise Robustness Techniques

Whisper achieves 95% accuracy at 10 dB SNR (very noisy). How?

### 1. **Data Augmentation During Training**
- Whisper's training data includes:
  - Street noise, music, background chatter
  - Reverberation (echoes in large rooms)
  - Codec artifacts (compressed audio, MP3)
- Model learns to ignore noise, focus on speech

### 2. **Preprocessing: Noise Reduction**
Even with Whisper, preprocessing helps:
- **Spectral Subtraction**: Estimate noise spectrum during silence, subtract from speech
- **Wiener Filtering**: Statistical filter based on SNR estimation
- **ROS 2 Integration**: Use `audio_common`'s `noise_reduction_node`

```python
from audio_common_msgs.msg import AudioStamped
import noisereduce as nr  # Python library

class NoiseReductionNode(Node):
    def __init__(self):
        super().__init__('noise_reduction')
        self.sub = self.create_subscription(AudioStamped, '/audio_raw', self.callback, 10)
        self.pub = self.create_publisher(AudioStamped, '/audio_clean', 10)

    def callback(self, msg):
        audio = np.frombuffer(msg.audio.data, dtype=np.int16).astype(np.float32)

        # Reduce noise (uses first 0.5s as noise profile)
        reduced = nr.reduce_noise(y=audio, sr=16000, stationary=True)

        clean_msg = AudioStamped()
        clean_msg.header = msg.header
        clean_msg.audio.data = (reduced * 32768).astype(np.int16).tobytes()
        self.pub.publish(clean_msg)
```

### 3. **Microphone Array Beamforming**
- **Problem**: Omnidirectional microphones pick up noise from all directions.
- **Solution**: Microphone array (4–8 mics) uses beamforming to focus on speaker direction.
- **Hardware**: ReSpeaker 4-Mic Array, Matrix Voice (8 mics)
- **Algorithm**: Delay-and-sum beamforming (constructive interference toward target, destructive elsewhere)

**ROS 2 Integration**: `respeaker_ros` package provides beamformed audio:
```bash
ros2 launch respeaker_ros respeaker.launch.py
# Publishes to /audio_beamformed (cleaner than /audio_raw)
```

### 4. **Confidence Scores**
Whisper outputs per-segment confidence:
```python
result = model.transcribe(audio)
for segment in result["segments"]:
    print(f"{segment['text']} (confidence: {segment['confidence']:.2f})")
# Output: "Go to the kitchen" (confidence: 0.94)
```

**Robotics Application**: Reject low-confidence transcriptions:
```python
if segment['confidence'] < 0.8:
    # Ask user to repeat
    tts_say("Sorry, I didn't catch that. Please repeat.")
```

## Multilingual Support

Whisper supports **99 languages** out-of-the-box. No fine-tuning required.

### Language Detection (Automatic)
```python
result = model.transcribe(audio, language=None)  # Auto-detect
print(f"Detected language: {result['language']}")
# Output: "Detected language: es" (Spanish)
```

### Explicit Language Specification
For faster inference (skip language detection), specify language:
```python
result = model.transcribe(audio, language="zh")  # Mandarin Chinese
```

### Translation to English
Whisper can translate non-English audio to English:
```python
result = model.transcribe(audio, task="translate")
# Input (Spanish): "Lleva esto a la cocina"
# Output (English): "Take this to the kitchen"
```

**Robotics Use Case**: Deploy single robot model globally; translate commands to English for LLM processing (LLMs perform better in English).

### Code-Switching
Whisper handles mixed-language sentences:
```python
# Input: "Robot, go to the biblioteca and bring me the libro"
result = model.transcribe(audio, language="es")
# Output: "Robot go to the biblioteca and bring me the libro"
# (Whisper preserves code-switching, doesn't force single language)
```

## ROS 2 Integration

### Architecture

```
┌─────────────────┐      ┌──────────────────┐      ┌──────────────────┐
│  Microphone     │─────▶│  audio_common    │─────▶│  whisper_node    │
│  (USB/I2S)      │ raw  │  (capture, noise │ clean│  (transcription) │
│                 │ audio│   reduction)     │ audio│                  │
└─────────────────┘      └──────────────────┘      └────────┬─────────┘
                                                             │
                                                             │ /transcription
                                                             ▼
                                                    ┌──────────────────┐
                                                    │   llm_planner    │
                                                    │   (task decomp)  │
                                                    └──────────────────┘
```

### Step 1: Install Dependencies

```bash
# Install Whisper
pip3 install openai-whisper

# Install audio_common (ROS 2)
sudo apt install ros-humble-audio-common

# Install ReSpeaker drivers (optional, for microphone array)
git clone https://github.com/respeaker/respeaker_ros.git ~/ros2_ws/src/
cd ~/ros2_ws && colcon build --packages-select respeaker_ros
```

### Step 2: Create Whisper ROS 2 Node

**File**: `whisper_node.py`
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from audio_common_msgs.msg import AudioStamped
import whisper
import numpy as np
import threading

class WhisperNode(Node):
    def __init__(self):
        super().__init__('whisper_transcriber')

        # Parameters
        self.declare_parameter('model_size', 'medium')
        self.declare_parameter('language', 'en')
        self.declare_parameter('confidence_threshold', 0.8)

        model_size = self.get_parameter('model_size').value
        self.language = self.get_parameter('language').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value

        # Load Whisper model
        self.get_logger().info(f"Loading Whisper model: {model_size}")
        self.model = whisper.load_model(model_size)
        self.get_logger().info("Whisper model loaded")

        # Audio buffer
        self.audio_buffer = []
        self.is_speaking = False
        self.lock = threading.Lock()

        # Subscribers and publishers
        self.audio_sub = self.create_subscription(
            AudioStamped, '/audio_clean', self.audio_callback, 10)
        self.transcription_pub = self.create_publisher(String, '/transcription', 10)

        self.get_logger().info("Whisper node ready")

    def audio_callback(self, msg):
        audio_chunk = np.frombuffer(msg.audio.data, dtype=np.int16)

        # Simple VAD: energy threshold
        energy = np.sqrt(np.mean(audio_chunk ** 2))

        with self.lock:
            if energy > 500:  # Speech detected
                self.is_speaking = True
                self.audio_buffer.extend(audio_chunk)
            elif self.is_speaking and energy < 500:
                # Silence after speech → transcribe
                if len(self.audio_buffer) > 8000:  # At least 0.5s
                    threading.Thread(target=self.transcribe).start()
                self.audio_buffer = []
                self.is_speaking = False

    def transcribe(self):
        with self.lock:
            audio_array = np.array(self.audio_buffer, dtype=np.float32) / 32768.0

        # Transcribe
        result = self.model.transcribe(
            audio_array,
            language=self.language,
            verbose=False
        )

        # Check confidence
        avg_confidence = np.mean([seg['confidence'] for seg in result['segments']])

        if avg_confidence >= self.confidence_threshold:
            msg = String()
            msg.data = result['text'].strip()
            self.transcription_pub.publish(msg)
            self.get_logger().info(f"Transcription: '{msg.data}' (conf: {avg_confidence:.2f})")
        else:
            self.get_logger().warn(f"Low confidence: {avg_confidence:.2f}, ignoring")

def main():
    rclpy.init()
    node = WhisperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 3: Launch File

**File**: `whisper_launch.py`
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Audio capture
        Node(
            package='audio_common',
            executable='audio_capturer_node',
            name='audio_capture',
            parameters=[{
                'device': 'hw:0,0',  # USB microphone
                'format': 'S16LE',
                'channels': 1,
                'rate': 16000
            }],
            remappings=[('audio', '/audio_raw')]
        ),

        # Whisper transcription
        Node(
            package='voice_control',
            executable='whisper_node.py',
            name='whisper',
            parameters=[{
                'model_size': 'medium',
                'language': 'en',
                'confidence_threshold': 0.8
            }]
        )
    ])
```

### Step 4: Run
```bash
# Terminal 1: Launch audio + Whisper
ros2 launch voice_control whisper_launch.py

# Terminal 2: Monitor transcriptions
ros2 topic echo /transcription

# Speak into microphone: "Go to the kitchen"
# Output: data: "Go to the kitchen"
```

## Performance Optimization

### GPU Acceleration
Whisper runs 10× faster on GPU:
```python
# Force GPU (CUDA)
model = whisper.load_model("medium", device="cuda")

# Verify GPU usage
import torch
print(f"Using device: {model.device}")  # Output: cuda:0
```

**Benchmark (3-second audio)**:
| Device          | Model  | Latency |
|-----------------|--------|---------|
| CPU (i7-12700K) | Medium | 2.1s    |
| GPU (RTX 3090)  | Medium | 0.21s   |
| Jetson Orin     | Medium | 0.35s   |

### Quantization (Faster Inference)
Use `faster-whisper` (CTranslate2 backend) for 4× speedup:
```bash
pip install faster-whisper
```

```python
from faster_whisper import WhisperModel

model = WhisperModel("medium", device="cuda", compute_type="int8")
segments, info = model.transcribe(audio, language="en")

for segment in segments:
    print(f"[{segment.start:.2f}s -> {segment.end:.2f}s] {segment.text}")
```

**Benchmark (int8 quantization)**:
- RTX 3090: 0.21s → **0.08s** (2.6× faster)
- Accuracy: 2.9% WER → 3.1% WER (negligible degradation)

## Summary

This chapter covered **Whisper speech recognition** for ROS 2 robotics:

- **Why Whisper?** Noise robust (95% at 10 dB SNR), multilingual (99 languages), open-source (Apache 2.0)
- **Architecture**: Encoder-decoder transformer (680M–1550M parameters), trained on 680,000 hours
- **Real-time vs. batch**: Batch mode simpler (700ms latency), streaming better for conversations
- **Noise robustness**: Preprocessing (noise reduction, beamforming), confidence thresholds
- **ROS 2 integration**: `whisper_node` subscribes to audio, publishes transcriptions
- **Performance**: GPU acceleration (10× faster), quantization (int8) for edge devices

Next chapter: **LLMs for cognitive planning**, where we use GPT-4/Claude to decompose voice commands into executable action sequences.

---

**Navigation**
← [Chapter 1: Voice-to-Action Pipelines](/vision-language-action/voice-to-action-pipelines)
→ [Chapter 3: LLMs for Cognitive Planning](/vision-language-action/llms-for-cognitive-planning)
