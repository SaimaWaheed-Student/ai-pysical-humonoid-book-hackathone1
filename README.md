# Book AI Assistant

This is a full-stack application that allows users to interact with book content using AI. The project consists of:

- **Backend API**: Built with FastAPI that handles document processing, embedding, and RAG (Retrieval Augmented Generation) functionality
- **Frontend Website**: A Docusaurus-based website in the `website` directory that provides the user interface

## Features

- Upload and process book documents
- Ask questions about the book content using AI
- Semantic search through book content
- Interactive chat interface

## Project Structure

```
hackatone-book-ai/
├── src/                 # FastAPI backend source code
├── website/             # Docusaurus frontend
├── requirements.txt     # Python dependencies
├── vercel.json          # Vercel deployment configuration
└── run_rag_chatbot.py   # Main application runner
```

## Local Development

### Backend (FastAPI)

1. Install Python dependencies:
```bash
pip install -r requirements.txt
```

2. Run the development server:
```bash
python run_rag_chatbot.py
```

The API will be available at `http://localhost:8000` with documentation at `http://localhost:8000/docs`.

### Frontend (Docusaurus)

1. Navigate to the website directory:
```bash
cd website
```

2. Install dependencies:
```bash
npm install
```

3. Run the development server:
```bash
npm start
```

The website will be available at `http://localhost:3000`.

## Deployment

### Backend to Vercel

The FastAPI backend can be deployed to Vercel using the `vercel-python` runtime.

### Frontend to Vercel

The Docusaurus website in the `website` directory is configured for deployment to Vercel via the `vercel.json` file.

## Environment Variables

Create a `.env` file based on `.env.example` with your API keys and configuration.