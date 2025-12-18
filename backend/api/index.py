"""
Vercel serverless function entry point for FastAPI
"""
from app.main import app

# Vercel requires a handler function
handler = app
