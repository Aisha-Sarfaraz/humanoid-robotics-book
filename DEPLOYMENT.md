# Deployment Guide - RAG Chatbot

This guide explains how to deploy the Humanoid Robotics Book website with the RAG chatbot to production using Replit and GitHub Pages.

## Architecture

- **Frontend**: GitHub Pages (static hosting)
- **Backend**: Replit (Python/FastAPI container hosting)
- **Vector Database**: Qdrant Cloud (free tier)
- **LLM**: Gemini API (primary) + OpenAI API (fallback)

---

## Step 1: Deploy Backend to Replit

### 1.1 Sign Up for Replit

1. **Go to Replit**: https://replit.com/
2. **Sign up** with your GitHub account (free, no credit card required)
3. Verify your email if prompted

### 1.2 Import Your Repository

1. Click **"+ Create Repl"**
2. Select **"Import from GitHub"**
3. Paste your repository URL: `https://github.com/Aisha-Sarfaraz/humanoid-robotics-book`
4. Replit will import the repository
5. Click **"Import from GitHub"** to confirm

### 1.3 Configure Environment Variables (Secrets)

In your Repl, click the **"Secrets"** tab (🔒 icon in left sidebar) and add these environment variables:

**Required:**
```env
GEMINI_API_KEY=your_actual_gemini_api_key
QDRANT_URL=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key
DATABASE_URL=your_postgres_connection_string
```

**Optional (for OpenAI fallback):**
```env
OPENAI_API_KEY=your_actual_openai_api_key
```

**Configuration:**
```env
LLM_PROVIDER=gemini
ENABLE_FALLBACK=true
GEMINI_EMBEDDING_MODEL=models/text-embedding-004
GEMINI_CHAT_MODEL=gemini-2.5-flash
QDRANT_COLLECTION_GEMINI=humanoid-robotics-gemini
QDRANT_COLLECTION_OPENAI=humanoid-robotic-book
```

### 1.4 Start the Backend

1. **Create `.replit` file** in the root directory with:
   ```toml
   run = "cd backend && pip install -r requirements.txt && uvicorn app.main:app --host 0.0.0.0 --port 8000"
   ```

2. **Click the "Run" button** (▶️ at the top)
3. Replit will:
   - Install dependencies from `backend/requirements.txt`
   - Start the FastAPI app on port 8000
   - Assign a URL like: `https://[repl-id].replit.dev`

4. Wait for server to start (~1-2 minutes)
5. Copy your Repl URL (shown in the webview pane)
6. Test the health endpoint:
   ```
   https://[your-repl-url]/api/v1/health
   ```

---

## Step 2: Get Your API Keys

### 2.1 Gemini API Key (Primary)

1. Go to https://aistudio.google.com/app/apikey
2. Create a new API key
3. Copy the key

### 2.2 Qdrant Cloud

1. Sign up at https://cloud.qdrant.io/
2. Create a free cluster
3. Get:
   - Cluster URL (looks like: `https://xxx-xxx-xxx.aws.cloud.qdrant.io`)
   - API Key from cluster settings

### 2.3 OpenAI API Key (Optional Fallback)

1. Go to https://platform.openai.com/api-keys
2. Create a new secret key
3. Copy the key

### 2.4 PostgreSQL Database (Neon Serverless)

1. Go to https://neon.tech/
2. Create a free project
3. Copy the connection string (starts with `postgresql://`)

---

## Step 3: Configure Frontend for Production

### 3.1 Update Environment Variable

Create `.env.production` file in the root:
```env
BACKEND_URL=https://[your-repl-url]
```

### 3.2 Add GitHub Secret

1. Go to your GitHub repository
2. **Settings** → **Secrets and variables** → **Actions**
3. Click **"New repository secret"**
4. Add:
   - Name: `BACKEND_URL`
   - Value: `https://[your-repl-url]` (your Replit backend URL)

---

## Step 4: Deploy Frontend to GitHub Pages

### 4.1 GitHub Actions Auto-Deploy

The workflow is already configured in `.github/workflows/deploy.yml`. It will:
- Build the Docusaurus site with your backend URL
- Deploy to GitHub Pages automatically

### 4.2 Trigger Deployment

**Option A: Automatic (Recommended)**
```bash
git add .
git commit -m "feat: configure Vercel backend URL"
git push origin main
```

GitHub Actions will automatically deploy.

**Option B: Manual**
1. Go to **Actions** tab
2. Select **"Deploy to GitHub Pages"**
3. Click **"Run workflow"**

---

## Step 5: Verify Deployment

### 5.1 Check Backend (Replit)

1. Visit: `https://[your-repl-url]/api/v1/health`
2. Should return:
   ```json
   {
     "status": "healthy",
     "service": "Humanoid Robotics Book RAG Chatbot API",
     "version": "1.0.0"
   }
   ```

### 5.2 Check Frontend (GitHub Pages)

1. Visit: `https://aisha-sarfaraz.github.io/humanoid-robotics-book/`
2. Look for the blue robot button (🤖) in the bottom-right corner
3. Click it and test the chatbot

### 5.3 Test Chatbot

1. Ask: "What is ROS 2?"
2. Should see:
   - Streaming response in real-time
   - Source citations
   - Smooth typing animation

---

## Step 6: Configure CORS

The CORS is already configured in `backend/app/main.py` to allow:
- GitHub Pages domain
- Localhost for development

If you need to add more origins, update the CORS middleware:

```python
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "https://aisha-sarfaraz.github.io",
        "http://localhost:3000",
        "https://your-custom-domain.com"  # Add your custom domain
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

---

## Replit Deployment Tips

### Free Tier Limitations

Replit Free Tier (Hacker Plan):
- **Always-On**: No (Repl sleeps after inactivity)
- **Auto-Wake**: Yes (wakes up on request, ~5-10 seconds delay)
- **Memory**: 500 MB
- **Storage**: 500 MB
- **Bandwidth**: Unlimited

**Note**: Streaming responses work perfectly. First request after sleep may take a few seconds.

### Keep Repl Always On (Optional)

**Option A: UptimeRobot (Free)**
1. Sign up at https://uptimerobot.com/
2. Add monitor: `https://[your-repl-url]/api/v1/health`
3. Set interval to 5 minutes
4. This pings your Repl to keep it awake

**Option B: Upgrade to Replit Hacker Plan** ($7/month)
- Always-on Repls
- More resources
- Faster performance

### Environment Variables (Secrets)

- Set in Replit: **Secrets tab** (🔒 icon)
- Automatically available to your app
- Secured and encrypted
- Changes take effect immediately (restart Repl)

### Logs and Monitoring

- View real-time logs in the **Console** tab
- Monitor resource usage in Repl dashboard
- Check errors in the output pane

### Custom Domain (Optional - Requires Paid Plan)

1. Upgrade to Replit Hacker Plan
2. Go to Repl settings
3. **Domains** → **Add Custom Domain**
4. Follow DNS configuration instructions

---

## Troubleshooting

### Backend Not Starting on Replit

**Issue**: Server crashes or fails to start

**Solutions**:
- Check Console tab for error messages
- Verify all Secrets are set correctly
- Ensure `backend/requirements.txt` has all dependencies
- Restart the Repl (click Stop then Run)
- Check if port 8000 is available

### Chatbot Button Not Showing

**Issue**: Frontend not loading ChatInterface

**Solutions**:
- Hard refresh: `Ctrl+Shift+R`
- Check browser console for errors (F12)
- Verify `BACKEND_URL` is set in GitHub secrets
- Check GitHub Actions build logs

### CORS Errors

**Issue**: "Access-Control-Allow-Origin" errors

**Solutions**:
- Verify CORS origins in `backend/app/main.py`
- Ensure GitHub Pages URL is included
- Redeploy backend after CORS changes

### Streaming Not Working

**Issue**: Response doesn't stream in real-time

**Solutions**:
- Check network tab: SSE connection should be established
- Verify backend URL is HTTPS (Replit provides this)
- Wait 5-10 seconds if Repl was sleeping (first request slower)
- Check Console tab in Replit for errors
- Ensure streaming endpoint is working: `/api/v1/chat/stream`
- Restart the Repl if needed

### API Quota Exceeded

**Issue**: Gemini API returns 429 errors

**Solutions**:
- Wait for quota reset (15 RPM free tier)
- Upgrade Gemini API plan
- Use OpenAI fallback (ensure `OPENAI_API_KEY` is set)

---

## Cost Estimate

### Free Tier Components:

- **GitHub Pages**: Free
- **Replit**: Free tier (auto-sleep, 500MB RAM/storage)
- **Qdrant Cloud**: 1GB storage free
- **Neon Serverless**: 0.5GB storage, 3GB data transfer free
- **Gemini API**: 15 RPM free tier

**Total Cost**: $0/month with free tiers

### Paid Upgrades (if needed):

- **Replit Hacker**: $7/month (always-on, more resources, custom domain)
- **Qdrant**: $25/month for 2GB+
- **Neon**: $19/month for higher limits
- **Gemini API**: Pay-as-you-go after free tier
- **UptimeRobot Pro**: $7/month (1-minute intervals, more monitors)

---

## Monitoring

### Replit Dashboard

- Monitor Repl status (running/sleeping)
- View Console logs in real-time
- Check resource usage (CPU, memory)
- Monitor active connections

### GitHub Actions

- Monitor deployment status
- View build logs
- Check deployment history

### Uptime Monitoring (Recommended for Free Tier)

- **UptimeRobot** (https://uptimerobot.com/) - Free
  - Add monitor: `https://[your-repl-url]/api/v1/health`
  - Interval: 5 minutes
  - Keeps Repl awake and monitors availability
  - Get email alerts if service goes down

---

## Updating the Application

### Updating Backend

**Option A: Update via Git (Recommended)**
1. Make changes to backend code locally
2. Push to GitHub:
   ```bash
   git add backend/
   git commit -m "fix: update backend logic"
   git push origin main
   ```
3. In Replit, pull the latest changes:
   - Open the Shell tab in Replit
   - Run: `git pull origin main`
   - Restart the Repl (click Stop then Run)

**Option B: Edit Directly in Replit**
1. Make changes in Replit's code editor
2. Changes auto-save
3. Restart the Repl to apply changes

### Updating Frontend

```bash
# Make changes to frontend code
git add src/
git commit -m "feat: update UI"
git push origin main
```

GitHub Actions auto-deploys on push to main branch.

### Re-indexing Documents

```bash
curl -X POST https://[your-repl-url]/api/v1/reindex
```

Or use the API endpoint with your frontend.

---

## Security Best Practices

1. ✅ Never commit `.env` files
2. ✅ Use Replit Secrets for all API keys and credentials
3. ✅ Enable rate limiting (already configured)
4. ✅ Use HTTPS only (Replit provides this automatically)
5. ✅ Keep dependencies updated
6. ✅ Monitor API usage and costs
7. ✅ Don't expose Secrets in code or logs
8. ✅ Regularly review Replit Console logs for suspicious activity

---

## Next Steps After Deployment

- [ ] Set up monitoring with UptimeRobot (keeps Repl awake + alerts)
- [ ] Configure custom domain (requires Replit Hacker plan)
- [ ] Add analytics (Google Analytics, Plausible)
- [ ] Set up automated backups for Qdrant data
- [ ] Implement usage tracking and rate limit adjustments
- [ ] Consider upgrading to Replit Hacker for always-on hosting ($7/month)

---

## Support

If you encounter issues:

1. Check Replit Console tab for errors
2. Check GitHub Actions logs
3. Review browser console errors (F12)
4. Verify all Secrets are set correctly
5. Restart the Repl
6. Refer to documentation:
   - Replit: https://docs.replit.com/
   - Docusaurus: https://docusaurus.io/docs
   - FastAPI: https://fastapi.tiangolo.com/

---

## Quick Start Checklist

- [x] Sign up for Replit (free, no credit card)
- [x] Import repository to Replit
- [x] Add all Secrets (13 environment variables)
- [x] Run the backend (click Run button)
- [x] Get backend URL from Replit
- [ ] Add `BACKEND_URL` to GitHub Secrets
- [ ] Push to main branch (triggers auto-deploy)
- [ ] Test chatbot on GitHub Pages
- [ ] Optional: Set up UptimeRobot to keep Repl awake

**Deployment Complete! 🎉**
