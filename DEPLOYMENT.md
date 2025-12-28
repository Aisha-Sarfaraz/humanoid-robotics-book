# Deployment Guide - RAG Chatbot

This guide explains how to deploy the Humanoid Robotics Book website with the RAG chatbot to production using Hugging Face Spaces and GitHub Pages.

## Architecture

- **Frontend**: GitHub Pages (static hosting)
- **Backend**: Hugging Face Spaces (Docker-based FastAPI hosting)
- **Vector Database**: Qdrant Cloud (free tier)
- **LLM**: Gemini API (primary) + OpenAI API (fallback)

---

## Quick Start Summary

1. Deploy backend to Hugging Face Spaces (10 minutes)
2. Update GitHub secret with backend URL
3. Frontend auto-deploys via GitHub Actions
4. Test chatbot on GitHub Pages

---

## Part 1: Backend Deployment (Hugging Face Spaces)

### Step 1: Create Hugging Face Account

1. Go to: https://huggingface.co/join
2. Sign up with email or GitHub
3. Verify your email

---

### Step 2: Create a New Space

1. Visit: https://huggingface.co/new-space
2. Fill in:
   - **Owner**: Your username
   - **Space name**: `humanoid-robotics-api`
   - **License**: Apache 2.0
   - **Select the Space SDK**: **Docker**
   - **Space hardware**: CPU basic (free)
3. Click **"Create Space"**

---

### Step 3: Clone and Push Backend Code

In your local terminal:

```bash
# Clone your new Hugging Face Space
git clone https://huggingface.co/spaces/YOUR_USERNAME/humanoid-robotics-api
cd humanoid-robotics-api

# Copy backend files
cp -r /path/to/humanoid-robotics-book/backend/* .

# Commit and push
git add .
git commit -m "Initial backend deployment"
git push
```

**Note**: Replace `YOUR_USERNAME` with your Hugging Face username.

---

### Step 4: Configure Environment Secrets

1. Go to your Space: `https://huggingface.co/spaces/YOUR_USERNAME/humanoid-robotics-api`
2. Click **"Settings"** tab
3. Scroll to **"Repository secrets"**
4. Add these secrets one by one:

```env
DATABASE_URL=postgresql://user:password@host/database
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_GEMINI=humanoid-robotics-gemini
QDRANT_COLLECTION_OPENAI=humanoid-robotic-book
LLM_PROVIDER=gemini
ENABLE_FALLBACK=true
GEMINI_API_KEY=your_gemini_api_key
GEMINI_EMBEDDING_MODEL=models/text-embedding-004
GEMINI_CHAT_MODEL=gemini-2.5-flash
OPENAI_API_KEY=your_openai_api_key
OPENAI_EMBEDDING_MODEL=text-embedding-3-small
OPENAI_CHAT_MODEL=gpt-4o-mini
```

5. Click **"Save"** for each secret

---

### Step 5: Wait for Build

1. Go to **"App"** tab in your Space
2. Wait 3-5 minutes for Docker build to complete
3. Once ready, you'll see: **"Running"** status

Your backend URL will be:
```
https://YOUR_USERNAME-humanoid-robotics-api.hf.space
```

---

### Step 6: Test Backend

Visit your health endpoint:
```
https://YOUR_USERNAME-humanoid-robotics-api.hf.space/api/v1/health
```

Should return:
```json
{
  "status": "healthy",
  "service": "Humanoid Robotics Book RAG Chatbot API",
  "version": "1.0.0"
}
```

---

## Part 2: Frontend Deployment (GitHub Pages)

### Step 1: Update GitHub Secret

1. Go to: https://github.com/YOUR_USERNAME/humanoid-robotics-book/settings/secrets/actions
2. Find or create `BACKEND_URL` secret
3. Update value to: `https://YOUR_USERNAME-humanoid-robotics-api.hf.space`
4. Click **"Update secret"**

---

### Step 2: Update CORS Settings

Update `backend/app/config.py` to allow your GitHub Pages domain:

```python
ALLOWED_ORIGINS: List[str] = [
    "http://localhost:3000",
    "https://YOUR_USERNAME.github.io",
]
```

Commit and push this change to trigger rebuild on Hugging Face.

---

### Step 3: Deploy Frontend

Push any change to trigger deployment:

```bash
git commit --allow-empty -m "trigger deployment"
git push origin main
```

Or manually trigger:
1. Go to: https://github.com/YOUR_USERNAME/humanoid-robotics-book/actions
2. Click **"Deploy to GitHub Pages"**
3. Click **"Run workflow"**

Wait 2-3 minutes for deployment.

---

### Step 4: Test Complete Application

1. Visit: `https://YOUR_USERNAME.github.io/humanoid-robotics-book/`
2. Click robot button (🤖) in bottom-right
3. Ask: "What is ROS 2?"
4. Should get instant streaming response!

---

## Troubleshooting

### Backend Build Fails

**Check Build Logs:**
1. Go to Space → **"App"** tab
2. Scroll down to see build logs
3. Look for error messages

**Common fixes:**
```bash
# Locally test Docker build
cd backend
docker build -t test-api .
docker run -p 7860:7860 --env-file .env test-api
```

### CORS Errors

Check `backend/app/config.py` has correct origins:

```python
ALLOWED_ORIGINS: List[str] = [
    "http://localhost:3000",
    "https://YOUR_USERNAME.github.io",
]
```

### Import Errors

Verify all dependencies in `backend/requirements.txt` are listed.

---

## Updating Backend

When you update backend code:

```bash
cd humanoid-robotics-api  # Your HF Space repo
git pull  # Get latest changes
# Make your changes
git add .
git commit -m "Update backend"
git push
```

Hugging Face will automatically rebuild and redeploy (takes 3-5 minutes).

---

## Hugging Face Spaces Benefits

✅ **Free Tier**: Generous free CPU resources
✅ **Always-On**: No cold starts or sleep mode
✅ **Docker Support**: Full control over environment
✅ **Auto-Deploy**: Git push triggers rebuild
✅ **Secrets Management**: Built-in environment variables
✅ **Public API**: Accessible from anywhere

**Free Tier Limits:**
- CPU: 2 vCPU
- RAM: 16 GB
- Storage: Persistent
- Always-on: ✅ Yes

---

## Monitoring

### Health Check
`https://YOUR_USERNAME-humanoid-robotics-api.hf.space/api/v1/health`

### API Documentation
`https://YOUR_USERNAME-humanoid-robotics-api.hf.space/api/v1/docs`

### Logs
View real-time logs in Space → **"Logs"** tab

### Optional: Uptime Monitoring
Set up UptimeRobot (free) to monitor availability:
1. https://uptimerobot.com/
2. Add monitor: Your health endpoint URL
3. Interval: 5 minutes

---

## Cost Comparison

| Platform | Free Tier | Always-On | Docker Support |
|----------|-----------|-----------|----------------|
| **Hugging Face Spaces** | ✅ Yes | ✅ Yes | ✅ Yes |
| PythonAnywhere | ✅ Yes | ✅ Yes | ❌ No |
| Render | ❌ No longer free | - | ✅ Yes |
| Vercel | ✅ Yes (Serverless) | ⚡ Serverless | ❌ No |

**Winner**: Hugging Face Spaces for FastAPI apps with Docker.

---

## Next Steps

- ✅ Backend deployed on Hugging Face Spaces
- ✅ Frontend connected to backend
- ✅ Chatbot working on GitHub Pages
- 🎉 **Production ready!**

Optional improvements:
- Set up uptime monitoring
- Monitor API usage in Hugging Face dashboard
- Consider upgrading to GPU for faster responses

---

## Support

**Hugging Face:**
- Documentation: https://huggingface.co/docs/hub/spaces
- Community: https://discuss.huggingface.co/

**Project Issues:**
- Check Space logs for errors
- Verify environment variables/secrets
- Test health endpoint first

---

**Your chatbot is now live and production-ready! 🚀**
