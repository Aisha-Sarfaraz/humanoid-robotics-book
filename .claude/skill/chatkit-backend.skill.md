# ChatKit-Backend Integration Skill

## Name
chatkit-backend

## Description
Comprehensive skill for implementing OpenAI ChatKit backends using the ChatKit Python SDK. Provides step-by-step guidance for building production-ready chat backends with FastAPI, response streaming via Server-Sent Events, tool integration, and session management.

## Use Cases
- Building custom chat backends for ChatKit frontend
- Implementing RAG (Retrieval-Augmented Generation) chat systems
- Creating AI assistants with custom business logic
- Integrating with existing databases and authentication systems
- Building chat backends for documentation sites
- Implementing tool-calling and function execution
- Creating streaming chat APIs with Server-Sent Events

## Prerequisites
- Python 3.8+ installed
- FastAPI or Flask knowledge
- OpenAI API key
- Basic understanding of async/await patterns
- Familiarity with REST APIs and SSE (Server-Sent Events)
- PostgreSQL or other database (for session management)

## Framework Support
- **Primary**: FastAPI (recommended for async support)
- **Compatible**: Flask, Django, any Python web framework
- **Database**: PostgreSQL, MySQL, SQLite (via SQLAlchemy or similar)
- **Streaming**: Server-Sent Events (SSE) for response streaming
- **Package**: `openai-chatkit` from PyPI

## Core Capabilities

### 1. ChatKit Server Implementation
- ChatKitServer base class for handling chat logic
- respond() method for processing user messages
- Event-based communication with frontend
- Session management and persistence
- Tool integration and function calling

### 2. Response Streaming
- Server-Sent Events (SSE) for real-time streaming
- Gradual text generation for natural UX
- Chunk-based streaming from OpenAI API
- Error handling during streaming
- Connection management

### 3. Tool Integration
- Define custom tools and functions
- Tool calling with OpenAI function calling
- Execute Python functions from chat
- Return tool results to LLM
- Handle tool errors gracefully

### 4. Backend Features
- Custom business logic integration
- Database query execution from chat
- Authentication and authorization
- Rate limiting and cost control
- Audit logging and monitoring
- File upload handling
- Multi-user session management

### 5. Integration Patterns
- FastAPI route setup
- Async/await for performance
- CORS configuration for frontend
- Environment variable management
- Error handling and logging

## Architecture Pattern

### ChatKit Backend Architecture
```
┌──────────────────────┐
│  ChatKit Frontend    │
│  (React/Vue/etc.)    │
└──────────┬───────────┘
           │ POST /chat
           │ (SSE streaming)
           ▼
┌──────────────────────┐
│  FastAPI Server      │
│  - ChatKitServer     │
│  - respond() method  │
│  - Tool definitions  │
└──────────┬───────────┘
           │
    ┌──────┴──────┐
    │             │
    ▼             ▼
┌─────────┐   ┌──────────┐
│Database │   │ OpenAI   │
│(Sessions│   │   API    │
│Messages)│   │(Streaming│
└─────────┘   └──────────┘
```

### FastAPI Integration Pattern
```python
from chatkit import ChatKitServer
from fastapi import FastAPI

class MyChat(ChatKitServer):
    async def respond(self, request):
        # Custom logic here
        response = await openai_stream(request.message)
        return response

app = FastAPI()
chat = MyChat()

@app.post("/chat")
async def handle_chat(request):
    return await chat.handle(request)
```

## Standard File Structure

```
backend/
├── app/
│   ├── chatkit/
│   │   ├── __init__.py
│   │   ├── server.py              # ChatKitServer implementation
│   │   ├── tools.py                # Tool definitions
│   │   └── handlers.py            # Custom message handlers
│   ├── routers/
│   │   └── chat.py                 # FastAPI routes
│   ├── models/
│   │   ├── session.py              # Session model
│   │   └── message.py              # Message model
│   ├── services/
│   │   └── openai_service.py       # OpenAI integration
│   ├── main.py                     # FastAPI app
│   └── config.py                   # Configuration
├── .env                            # Environment variables
└── requirements.txt                # Python dependencies
```

## Configuration Templates

### Basic ChatKit Server (FastAPI)
```python
# app/chatkit/server.py
from chatkit import ChatKitServer, Message
from openai import AsyncOpenAI
import os

client = AsyncOpenAI(api_key=os.getenv("OPENAI_API_KEY"))

class MyChatServer(ChatKitServer):
    async def respond(self, request):
        """Handle user messages and stream responses"""
        messages = request.messages

        # Call OpenAI with streaming
        stream = await client.chat.completions.create(
            model="gpt-4o",
            messages=[{"role": m.role, "content": m.content} for m in messages],
            stream=True
        )

        # Stream responses back
        async for chunk in stream:
            if chunk.choices[0].delta.content:
                yield Message(
                    role="assistant",
                    content=chunk.choices[0].delta.content
                )
```

### FastAPI Route Setup
```python
# app/routers/chat.py
from fastapi import APIRouter, HTTPException
from fastapi.responses import StreamingResponse
from ..chatkit.server import MyChatServer

router = APIRouter(prefix="/api/v1")
chat_server = MyChatServer()

@router.post("/chat")
async def chat_endpoint(request: ChatRequest):
    """Handle chat requests with SSE streaming"""
    try:
        return StreamingResponse(
            chat_server.handle(request),
            media_type="text/event-stream"
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
```

### Main FastAPI App
```python
# app/main.py
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from .routers import chat
from .config import settings

app = FastAPI(title="ChatKit Backend")

# CORS for frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.ALLOWED_ORIGINS,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"]
)

app.include_router(chat.router)

@app.get("/health")
async def health_check():
    return {"status": "ok"}
```

### With Tool Calling
```python
# app/chatkit/server.py
from chatkit import ChatKitServer, Message, Tool

class MyToolChatServer(ChatKitServer):
    def __init__(self):
        self.tools = [
            Tool(
                name="search_docs",
                description="Search documentation",
                parameters={
                    "type": "object",
                    "properties": {
                        "query": {"type": "string"}
                    }
                },
                function=self.search_docs
            )
        ]

    async def search_docs(self, query: str):
        """Execute documentation search"""
        # Your search logic here
        results = await semantic_search(query)
        return {"results": results}

    async def respond(self, request):
        stream = await client.chat.completions.create(
            model="gpt-4o",
            messages=request.messages,
            tools=[t.to_dict() for t in self.tools],
            stream=True
        )

        async for chunk in stream:
            # Handle tool calls
            if chunk.choices[0].delta.tool_calls:
                tool_call = chunk.choices[0].delta.tool_calls[0]
                result = await self.execute_tool(tool_call)
                yield Message(role="tool", content=result)

            # Handle text content
            if chunk.choices[0].delta.content:
                yield Message(
                    role="assistant",
                    content=chunk.choices[0].delta.content
                )
```

## Environment Variables

```env
# OpenAI Configuration
OPENAI_API_KEY=sk-...
OPENAI_MODEL=gpt-4o

# Database
DATABASE_URL=postgresql://user:pass@localhost/chatkit

# Server
PORT=8000
HOST=0.0.0.0
ALLOWED_ORIGINS=http://localhost:3000,https://yourdomain.com

# Optional: Rate Limiting
RATE_LIMIT_PER_MINUTE=60

# Optional: Logging
LOG_LEVEL=INFO
```

## Common Integration Scenarios

### Scenario 1: Basic Chat Backend
**Steps**:
1. Install: `pip install openai-chatkit fastapi uvicorn`
2. Create ChatKitServer subclass
3. Implement respond() method
4. Set up FastAPI route
5. Configure CORS for frontend
6. Test with ChatKit frontend

**Use case**: Simple chat backend for customer support widget.

### Scenario 2: RAG Chat Backend
**Steps**:
1. Set up vector database (Qdrant, Pinecone)
2. Create document embedding pipeline
3. Implement semantic search in respond()
4. Add context injection to OpenAI calls
5. Return sources with citations
6. Stream responses with context

**Use case**: Knowledge base chatbot for internal wiki or product docs.

### Scenario 3: Tool-Calling Backend
**Steps**:
1. Define tools with parameters
2. Implement tool execution functions
3. Configure OpenAI function calling
4. Handle tool calls in respond()
5. Return tool results to LLM
6. Stream final response

**Use case**: AI assistant with database queries, API calls, calculations.

### Scenario 4: Multi-User Chat with Auth
**Steps**:
1. Add authentication middleware
2. Store sessions per user in database
3. Validate user ownership of sessions
4. Implement rate limiting per user
5. Add audit logging
6. Handle user context in respond()

**Use case**: Multi-tenant SaaS application with isolated conversations.

## Validation Checklist

Before considering ChatKit backend complete:

- [ ] ChatKitServer class implemented correctly
- [ ] respond() method handles messages
- [ ] FastAPI route returns StreamingResponse
- [ ] CORS configured for frontend origin
- [ ] Response streaming works (SSE format correct)
- [ ] OpenAI API key configured and valid
- [ ] Database connection established (if using)
- [ ] Tool calling works (if implemented)
- [ ] Error handling for OpenAI API failures
- [ ] Rate limiting configured
- [ ] Logging set up for debugging
- [ ] Environment variables loaded correctly

## Troubleshooting

### Issue: Streaming not working
**Symptoms**: Frontend doesn't show gradual text, receives all at once

**Solutions**:
1. Ensure FastAPI route returns `StreamingResponse`
2. Set `media_type="text/event-stream"`
3. Verify CORS allows streaming
4. Check browser supports EventSource
5. Test with `curl -N` to verify SSE format

### Issue: OpenAI API errors
**Symptoms**: 401 Unauthorized, rate limit errors

**Solutions**:
1. Verify OPENAI_API_KEY is correct
2. Check API key has sufficient credits
3. Implement exponential backoff for rate limits
4. Monitor OpenAI API status page
5. Add error handling in respond()

### Issue: CORS errors
**Symptoms**: Frontend can't connect, preflight fails

**Solutions**:
1. Add frontend origin to `allow_origins`
2. Enable `allow_credentials=True`
3. Include all necessary headers in `allow_headers`
4. Handle OPTIONS requests properly

### Issue: Tool calling not working
**Symptoms**: Tools defined but not called

**Solutions**:
1. Verify tool schema matches OpenAI spec
2. Check function names are valid Python identifiers
3. Ensure `tools` parameter passed to OpenAI API
4. Handle tool_calls in response stream
5. Return tool results with correct format

## Documentation Resources

- **ChatKit Python SDK**: https://openai.github.io/chatkit-python/server/
- **OpenAI ChatKit Guide**: https://platform.openai.com/docs/guides/chatkit
- **Advanced Samples**: https://github.com/openai/openai-chatkit-advanced-samples
- **FastAPI Docs**: https://fastapi.tiangolo.com/
- **OpenAI Python SDK**: https://github.com/openai/openai-python
- **Streaming Guide**: https://platform.openai.com/docs/api-reference/streaming
- **Function Calling**: https://platform.openai.com/docs/guides/function-calling

## Usage Instructions

To invoke this skill in Claude Code, use:

```
Help me build a ChatKit backend with FastAPI
Implement ChatKit server with tool calling
Add RAG functionality to my ChatKit backend
Debug my ChatKit streaming issues
```

## Version Compatibility

- **openai-chatkit**: 1.4.0+
- **FastAPI**: 0.100.0+
- **Python**: 3.8+
- **openai**: 1.0.0+ (OpenAI Python SDK)
- **uvicorn**: 0.20.0+ (ASGI server)

## Notes

- ChatKitServer base class requires implementing respond() method
- Streaming uses Server-Sent Events (SSE) format
- Tool calling requires OpenAI function calling setup
- Production should use proper authentication
- Monitor OpenAI API costs with usage tracking
- Consider caching responses for identical queries
- Use async/await throughout for better performance
