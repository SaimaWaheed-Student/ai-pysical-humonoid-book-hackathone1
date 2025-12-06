import rclpy
from rclpy.node import Node
from std_msgs.msg import String
# from audio_common_msgs.msg import AudioData # Uncomment if you have audio_common_msgs

class WhisperTranscriber(Node):
    def __init__(self):
        super().__init__('whisper_transcriber_node')
        self.publisher_ = self.create_publisher(String, 'transcribed_text', 10)
        # self.subscription = self.create_subscription(
        #     AudioData,
        #     'audio_input',
        #     self.audio_callback,
        #     10)
        self.timer = self.create_timer(5.0, self.timer_callback) # Simulate transcription every 5 seconds
        self.wake_word_detected = False
        self.get_logger().info('Whisper Transcriber Node (Placeholder) started. Waiting for wake word.')

    # def audio_callback(self, msg: AudioData):
    #     # In a real scenario, this would process audio data using Whisper
    #     # For now, we simulate a transcription
    #     self.get_logger().info('Received dummy audio data. Simulating transcription.')
    #     transcribed_text = "Simulated: Robot, move forward."
    #     self.publish_transcription(transcribed_text)

    def timer_callback(self):
        # This timer simulates receiving audio and transcribing it
        dummy_audio_input = "User: 'Hey Robot, pick up the ball.'" # Simulate wake word
        # dummy_audio_input = "User: 'Just talking normally.'" # Simulate no wake word

        self.get_logger().info(f'Simulating audio input: "{dummy_audio_input}"')
        
        # Placeholder for actual wake word detection (e.g., using Porcupine)
        if "Hey Robot" in dummy_audio_input:
            self.wake_word_detected = True
            self.get_logger().info("Wake word 'Hey Robot' detected!")
        else:
            self.wake_word_detected = False
            self.get_logger().info("No wake word detected. Ignoring command.")
            return

        if self.wake_word_detected:
            # Placeholder for actual Whisper API call or local model inference
            transcribed_text = dummy_audio_input.replace("User: 'Hey Robot, ", "").replace("'", "")
            self.publish_transcription(transcribed_text)
            self.wake_word_detected = False # Reset after command



    def publish_transcription(self, text: str):
        msg = String()
        msg.data = text
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing transcription: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    node = WhisperTranscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
