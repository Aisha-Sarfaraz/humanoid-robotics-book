---
title: Humanoid Robotics RAG API
emoji: 🤖
colorFrom: blue
colorTo: purple
sdk: docker
pinned: false
license: cc-by-nc-sa-4.0
app_port: 7860
---

# Humanoid Robotics Book RAG Chatbot API

This is the backend API for the Humanoid Robotics Book RAG chatbot, built with FastAPI.

## Features

- **RAG (Retrieval-Augmented Generation)** for contextual responses
- **Gemini API** (primary) with **OpenAI API** fallback
- **Qdrant** vector database for semantic search
- **PostgreSQL** database for chat history
- **Rate limiting** and **streaming responses**

## API Endpoints

- `GET /api/v1/health` - Health check endpoint
- `POST /api/v1/chat/query` - Chat query endpoint
- `GET /api/v1/docs` - Interactive API documentation
- `GET /api/v1/redoc` - ReDoc API documentation

## Environment Variables

Create a `.env` file with the following variables:

```env
# Database
DATABASE_URL=postgresql://user:password@host/database

# Vector Database
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_GEMINI=humanoid-robotics-gemini
QDRANT_COLLECTION_OPENAI=humanoid-robotic-book

# LLM Provider
LLM_PROVIDER=gemini
ENABLE_FALLBACK=true

# Gemini API
GEMINI_API_KEY=your_gemini_api_key
GEMINI_EMBEDDING_MODEL=models/text-embedding-004
GEMINI_CHAT_MODEL=gemini-2.5-flash

# OpenAI API (Fallback)
OPENAI_API_KEY=your_openai_api_key
OPENAI_EMBEDDING_MODEL=text-embedding-3-small
OPENAI_CHAT_MODEL=gpt-4o-mini
```

## Local Development

### Install Dependencies

```bash
pip install -r requirements.txt
```

### Run Server

```bash
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

Visit http://localhost:8000/api/v1/docs for interactive API documentation.

## Docker Deployment

### Build Image

```bash
docker build -t humanoid-robotics-api .
```

### Run Container

```bash
docker run -p 7860:7860 --env-file .env humanoid-robotics-api
```

## Hugging Face Spaces Deployment

This API is configured for deployment on Hugging Face Spaces with Docker runtime.

### Steps:

1. Create a new Space on Hugging Face (https://huggingface.co/spaces)
2. Select "Docker" as the Space SDK
3. Clone this repository or upload the `backend` folder
4. Add environment variables in Space Settings → Repository secrets
5. The Space will automatically build and deploy

### Required Secrets:

Add these in your Hugging Face Space settings:

- `DATABASE_URL`
- `QDRANT_URL`
- `QDRANT_API_KEY`
- `GEMINI_API_KEY`
- `OPENAI_API_KEY`

## License

CC-BY-NC-SA-4.0
