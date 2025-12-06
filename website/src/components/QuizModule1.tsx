import React, { useState } from 'react';

interface Question {
  question: string;
  options: string[];
  answer: number; // Index of the correct option
}

const questions: Question[] = [
  {
    question: 'What is the primary communication mechanism in ROS 2 for asynchronous data exchange?',
    options: ['Services', 'Actions', 'Topics', 'Parameters'],
    answer: 2,
  },
  {
    question: 'Which tool is used to build ROS 2 packages?',
    options: ['Catkin', 'Colcon', 'Make', 'Ament'],
    answer: 1,
  },
  {
    question: 'What does URDF stand for?',
    options: ['Universal Robot Definition Format', 'Unified Robot Description Format', 'User Robot Design File', 'Utility Robotics Data Format'],
    answer: 1,
  },
  {
    question: 'ROS 2 uses which technology for decentralized communication?',
    options: ['TCP/IP', 'UDP', 'DDS', 'HTTP'],
    answer: 2,
  },
  {
    question: 'Which programming languages are primarily supported by ROS 2 client libraries?',
    options: ['Java and C#', 'Python and C++', 'Rust and Go', 'JavaScript and TypeScript'],
    answer: 1,
  },
  {
    question: 'What is the purpose of a ROS 2 Node?',
    options: ['To manage all robot hardware', 'To perform a specific computation', 'To store sensor data', 'To visualize robot status'],
    answer: 1,
  },
  {
    question: 'How do you represent the physical and kinematic properties of a robot in ROS?',
    options: ['YAML', 'JSON', 'URDF', 'XML'],
    answer: 2,
  },
  {
    question: 'What is the advantage of ROS 2 over ROS 1 regarding real-time performance?',
    options: ['ROS 1 has better real-time support', 'ROS 2 was designed with real-time capabilities in mind', 'Neither supports real-time operations', 'Only specific hardware can use ROS 2 real-time'],
    answer: 1,
  },
  {
    question: 'Which type of ROS 2 communication is used for long-running tasks with periodic feedback?',
    options: ['Topics', 'Services', 'Actions', 'Parameters'],
    answer: 2,
  },
  {
    question: 'What does the `rclpy` library provide?',
    options: ['ROS 2 client library for C++', 'ROS 2 client library for Python', 'ROS 2 message serialization', 'ROS 2 communication over DDS'],
    answer: 1,
  },
];

export default function QuizModule1(): JSX.Element {
  const [currentQuestion, setCurrentQuestion] = useState(0);
  const [selectedOption, setSelectedOption] = useState<number | null>(null);
  const [score, setScore] = useState(0);
  const [showResult, setShowResult] = useState(false);

  const handleOptionClick = (optionIndex: number) => {
    setSelectedOption(optionIndex);
  };

  const handleNextQuestion = () => {
    if (selectedOption === questions[currentQuestion].answer) {
      setScore(score + 1);
    }
    setSelectedOption(null);
    if (currentQuestion < questions.length - 1) {
      setCurrentQuestion(currentQuestion + 1);
    } else {
      setShowResult(true);
    }
  };

  const handleRestartQuiz = () => {
    setCurrentQuestion(0);
    setSelectedOption(null);
    setScore(0);
    setShowResult(false);
  };

  return (
    <div style={{ padding: '20px', maxWidth: '700px', margin: 'auto', backgroundColor: 'var(--ifm-background-color)', borderRadius: '8px', boxShadow: '0 4px 8px rgba(0,0,0,0.1)' }}>
      {showResult ? (
        <div>
          <h2>Quiz Result</h2>
          <p>You scored {score} out of {questions.length}!</p>
          <button onClick={handleRestartQuiz} className="button button--primary">Restart Quiz</button>
        </div>
      ) : (
        <div>
          <h2>Question {currentQuestion + 1} of {questions.length}</h2>
          <p>{questions[currentQuestion].question}</p>
          <div style={{ display: 'flex', flexDirection: 'column', gap: '10px' }}>
            {questions[currentQuestion].options.map((option, index) => (
              <button
                key={index}
                onClick={() => handleOptionClick(index)}
                style={{
                  backgroundColor: selectedOption === index ? 'var(--ifm-color-primary-light)' : 'var(--ifm-color-emphasis-100)',
                  color: selectedOption === index ? 'white' : 'var(--ifm-font-color-base)',
                  borderColor: 'var(--ifm-color-primary)',
                  borderWidth: '1px',
                  borderStyle: 'solid',
                  padding: '10px',
                  borderRadius: '5px',
                  cursor: 'pointer',
                  textAlign: 'left',
                }}
                className="button button--secondary"
              >
                {option}
              </button>
            ))}
          </div>
          <button
            onClick={handleNextQuestion}
            disabled={selectedOption === null}
            style={{ marginTop: '20px' }}
            className="button button--primary"
          >
            {currentQuestion < questions.length - 1 ? 'Next Question' : 'Finish Quiz'}
          </button>
        </div>
      )}
    </div>
  );
}
