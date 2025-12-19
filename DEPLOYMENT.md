# Deployment Guide - RAG Chatbot

This guide explains how to deploy the Humanoid Robotics Book website with the RAG chatbot to production using PythonAnywhere and GitHub Pages.

## Architecture

- **Frontend**: GitHub Pages (static hosting)
- **Backend**: PythonAnywhere (Always-On Python/FastAPI hosting)
- **Vector Database**: Qdrant Cloud (free tier)
- **LLM**: Gemini API (primary) + OpenAI API (fallback)

---

## Quick Start Summary

1. Deploy backend to PythonAnywhere (15 minutes)
2. Update GitHub secret with backend URL
3. Frontend auto-deploys via GitHub Actions
4. Test chatbot on GitHub Pages

---

## Step 1: Sign Up for PythonAnywhere

1. Go to: https://www.pythonanywhere.com/registration/register/beginner/
2. Choose a username (e.g., `aishasarfaraz`)
3. Enter email and password
4. Click **"Register"**
5. Verify your email and log in

---

## Step 2: Clone Repository

1. In PythonAnywhere dashboard, click **"Consoles"** → **"Bash"**
2. Run these commands:

```bash
git clone https://github.com/Aisha-Sarfaraz/humanoid-robotics-book.git
cd humanoid-robotics-book/backend
ls -la
```

---

## Step 3: Create Virtual Environment

```bash
cd ~/humanoid-robotics-book/backend
python3.10 -m venv venv
source venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

Wait 2-3 minutes for installation to complete.

---

## Step 4: Configure Environment Variables

Create `.env` file:

```bash
cd ~/humanoid-robotics-book/backend
nano .env
```

Paste your environment variables:

```env
# Database - Neon Postgres
DATABASE_URL=postgresql://user:password@host/database

# Vector Database - Qdrant Cloud
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

Save with: `Ctrl+O`, `Enter`, `Ctrl+X`

---

## Step 5: Create Web App

1. Go to **"Web"** tab in PythonAnywhere dashboard
2. Click **"Add a new web app"**
3. Click **"Next"**
4. Select **"Manual configuration"**
5. Select **"Python 3.10"**
6. Click **"Next"**

---

## Step 6: Configure WSGI File

1. On Web tab, find **"Code"** section
2. Click the **WSGI configuration file** link
3. **Delete all content** and paste:

```python
import sys
import os
from dotenv import load_dotenv

# Add project directory to sys.path
project_home = '/home/YOUR_USERNAME/humanoid-robotics-book/backend'
if project_home not in sys.path:
    sys.path = [project_home] + sys.path

# Load environment variables
load_dotenv(os.path.join(project_home, '.env'))

# Import FastAPI app
from app.main import app as application
```

4. **Replace `YOUR_USERNAME`** with your PythonAnywhere username
5. Click **"Save"**

---

## Step 7: Set Virtual Environment

1. On Web tab, scroll to **"Virtualenv"** section
2. Enter: `/home/YOUR_USERNAME/humanoid-robotics-book/backend/venv`
3. Press Enter

---

## Step 8: Start the Backend

1. Scroll to top of Web tab
2. Click big green **"Reload"** button
3. Wait 10-20 seconds

Your backend URL: `https://YOUR_USERNAME.pythonanywhere.com`

---

## Step 9: Test Backend

Visit: `https://YOUR_USERNAME.pythonanywhere.com/api/v1/health`

Should return:

```json
{
  "status": "healthy",
  "service": "Humanoid Robotics Book RAG Chatbot API",
  "version": "1.0.0"
}
```

If you see errors, check the **Error log** on Web tab.

---

## Step 10: Update GitHub Secret

1. Go to: https://github.com/Aisha-Sarfaraz/humanoid-robotics-book/settings/secrets/actions
2. Find `BACKEND_URL` secret
3. Click **"Update"** (pencil icon)
4. Set value: `https://YOUR_USERNAME.pythonanywhere.com`
5. Click **"Update secret"**

---

## Step 11: Deploy Frontend

Push any change to trigger deployment:

```bash
git commit --allow-empty -m "trigger deployment"
git push origin main
```

Or manually trigger:
1. Go to: https://github.com/Aisha-Sarfaraz/humanoid-robotics-book/actions
2. Click **"Deploy to GitHub Pages"**
3. Click **"Run workflow"**

Wait 2-3 minutes for deployment.

---

## Step 12: Test Chatbot

1. Visit: https://aisha-sarfaraz.github.io/humanoid-robotics-book/
2. Click robot button (🤖) in bottom-right
3. Ask: "What is ROS 2?"
4. Should get instant streaming response!

---

## Troubleshooting

### Backend Not Starting

**Check Error Log:**
1. Web tab → **"Error log"**
2. Look for recent errors

**Common fixes:**
```bash
cd ~/humanoid-robotics-book/backend
source venv/bin/activate
pip install -r requirements.txt
```

Then reload web app.

### CORS Errors

Check `backend/app/config.py` has:

```python
ALLOWED_ORIGINS: List[str] = [
    "http://localhost:3000",
    "https://aisha-sarfaraz.github.io",
]
```

### Import Errors

Verify WSGI file path matches your username:
```python
project_home = '/home/YOUR_USERNAME/humanoid-robotics-book/backend'
```

---

## Updating Backend

When you update backend code:

```bash
cd ~/humanoid-robotics-book/backend
git pull origin main
source venv/bin/activate
pip install -r requirements.txt  # If dependencies changed
```

Then: Web tab → Click **"Reload"**

---

## PythonAnywhere Benefits

✅ **Always-On**: Runs 24/7, no auto-sleep
✅ **Free**: No credit card required
✅ **Fast**: Instant responses, no wake-up delay
✅ **Reliable**: Production-ready Python hosting
✅ **Simple**: Deploy once, runs forever

**Free Tier Limits:**
- 100 seconds/day CPU time
- 512 MB storage
- Always-on web app
- HTTPS included

---

## Monitoring

### Health Check
`https://YOUR_USERNAME.pythonanywhere.com/api/v1/health`

### Logs
Available on Web tab:
- **Error log**: Python errors
- **Server log**: Requests/responses
- **Access log**: All HTTP requests

### Optional: Uptime Monitoring
Set up UptimeRobot (free) to monitor availability:
1. https://uptimerobot.com/
2. Add monitor: `https://YOUR_USERNAME.pythonanywhere.com/api/v1/health`
3. Interval: 5 minutes

---

## Cost Comparison

| Platform | Free Tier | Always-On | Wake-up Delay |
|----------|-----------|-----------|---------------|
| **PythonAnywhere** | ✅ Yes | ✅ Yes | ⚡ None |
| Replit | ✅ Yes | ❌ No | 🐌 5-10s |
| Render | ❌ No longer free | - | - |
| Vercel | ✅ Yes | ⚡ Serverless | ~1s cold start |

**Winner**: PythonAnywhere for this use case.

---

## Next Steps

- ✅ Backend deployed and always-on
- ✅ Frontend connected to backend
- ✅ Chatbot working on GitHub Pages
- 🎉 **Production ready!**

Optional improvements:
- Set up uptime monitoring
- Monitor API usage
- Consider paid tier ($5/month) for more CPU time

---

## Support

**PythonAnywhere:**
- Forums: https://www.pythonanywhere.com/forums/
- Help: https://help.pythonanywhere.com/

**Project Issues:**
- Check Error logs on Web tab
- Verify environment variables in `.env`
- Test health endpoint first

---

**Your chatbot is now live and production-ready! 🚀**
