# Deployment Guide - RAG Chatbot

This guide explains how to deploy the Humanoid Robotics Book website with the RAG chatbot to production.

## Architecture

- **Frontend**: GitHub Pages (static hosting)
- **Backend**: Render.com (Python/FastAPI hosting)
- **Vector Database**: Qdrant Cloud (free tier)
- **LLM**: Gemini API (primary) + OpenAI API (fallback)

---

## Step 1: Deploy Backend to Render

### 1.1 Create Render Account

1. Go to https://render.com/
2. Sign up with your GitHub account
3. Authorize Render to access your repository

### 1.2 Create New Web Service

1. Click **"New +"** → **"Web Service"**
2. Connect your GitHub repository: `humanoid-robotics-book`
3. Render will auto-detect the `render.yaml` configuration file

### 1.3 Configure Environment Variables

In the Render dashboard, add these environment variables:

**Required:**
```
GEMINI_API_KEY=your_actual_gemini_api_key
QDRANT_URL=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key
DATABASE_URL=your_postgres_connection_string
```

**Optional (for OpenAI fallback):**
```
OPENAI_API_KEY=your_actual_openai_api_key
```

**Configuration (already set in render.yaml):**
```
LLM_PROVIDER=gemini
ENABLE_FALLBACK=true
GEMINI_EMBEDDING_MODEL=models/text-embedding-004
GEMINI_CHAT_MODEL=gemini-2.5-flash
QDRANT_COLLECTION_GEMINI=humanoid-robotics-gemini
QDRANT_COLLECTION_OPENAI=humanoid-robotic-book
```

### 1.4 Deploy

1. Click **"Create Web Service"**
2. Render will automatically:
   - Install dependencies from `requirements.txt`
   - Start the FastAPI server
   - Assign a URL like: `https://humanoid-robotics-rag-backend.onrender.com`

3. Wait for deployment to complete (~5-10 minutes)
4. Test the health endpoint:
   ```
   https://your-app-name.onrender.com/api/v1/health
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

### 2.4 PostgreSQL Database (for Neon Serverless)

1. Go to https://neon.tech/
2. Create a free project
3. Copy the connection string (starts with `postgresql://`)

---

## Step 3: Configure Frontend for Production

### 3.1 Update GitHub Pages Deployment

1. Create a `.env.production` file:
   ```bash
   BACKEND_URL=https://your-app-name.onrender.com
   ```

2. Update `docusaurus.config.js` (already configured):
   ```javascript
   customFields: {
     backendUrl: process.env.BACKEND_URL || 'http://localhost:8000',
   }
   ```

### 3.2 GitHub Actions Workflow

Create `.github/workflows/deploy.yml`:

```yaml
name: Deploy to GitHub Pages

on:
  push:
    branches:
      - main

jobs:
  deploy:
    runs-on: ubuntu-latest

    steps:
      - uses: actions/checkout@v4

      - name: Setup Node.js
        uses: actions/setup-node@v4
        with:
          node-version: '18'
          cache: 'npm'

      - name: Install dependencies
        run: npm ci

      - name: Build website
        env:
          BACKEND_URL: ${{ secrets.BACKEND_URL }}
        run: npm run build

      - name: Deploy to GitHub Pages
        uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./build
```

### 3.3 Add GitHub Secrets

1. Go to your GitHub repository
2. Settings → Secrets and variables → Actions
3. Click **"New repository secret"**
4. Add:
   - Name: `BACKEND_URL`
   - Value: `https://your-app-name.onrender.com`

---

## Step 4: Deploy Frontend to GitHub Pages

### 4.1 Enable GitHub Pages

1. Go to repository Settings
2. Pages → Source → GitHub Actions

### 4.2 Deploy

**Option A: Manual Deploy**
```bash
npm run build
npm run deploy
```

**Option B: Automatic (via GitHub Actions)**
```bash
git add .
git commit -m "feat: configure production deployment"
git push origin main
```

GitHub Actions will automatically build and deploy.

---

## Step 5: Verify Deployment

### 5.1 Check Backend

1. Visit: `https://your-app-name.onrender.com/api/v1/health`
2. Should return:
   ```json
   {
     "status": "healthy",
     "service": "Humanoid Robotics Book RAG Chatbot API",
     "version": "1.0.0"
   }
   ```

### 5.2 Check Frontend

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

## Step 6: Configure CORS (if needed)

If you get CORS errors, update `backend/app/main.py`:

```python
from fastapi.middleware.cors import CORSMiddleware

app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "https://aisha-sarfaraz.github.io",
        "http://localhost:3000"
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

---

## Troubleshooting

### Backend Not Starting on Render

- Check Render logs: Dashboard → Logs
- Verify environment variables are set
- Ensure Python version is 3.13

### Chatbot Button Not Showing

- Hard refresh: `Ctrl+Shift+R`
- Check browser console for errors (F12)
- Verify `BACKEND_URL` environment variable

### CORS Errors

- Add your GitHub Pages URL to CORS origins
- Ensure backend is using HTTPS (Render provides this automatically)

### Streaming Not Working

- Check network tab in browser dev tools
- Verify SSE connection is established
- Check backend logs for errors

---

## Cost Estimate

### Free Tier Components:
- **GitHub Pages**: Free
- **Render.com**: 750 hours/month free (enough for 1 service 24/7)
- **Qdrant Cloud**: 1GB storage free
- **Neon Serverless**: 0.5GB storage, 3GB data transfer free
- **Gemini API**: 15 RPM free tier

### Paid Upgrades (if needed):
- **Render Pro**: $7/month (faster cold starts, more RAM)
- **Qdrant**: $25/month for 2GB+
- **Neon**: $19/month for higher limits
- **Gemini API**: Pay-as-you-go after free tier

---

## Monitoring

### Render Dashboard
- Monitor backend health
- View request logs
- Check resource usage

### GitHub Actions
- Monitor deployment status
- View build logs

### Uptime Monitoring (Optional)
- Use https://uptimerobot.com/ (free)
- Monitor backend health endpoint
- Get alerts if service goes down

---

## Maintenance

### Updating Backend
```bash
git push origin main
```
Render auto-deploys on push to main branch.

### Updating Frontend
```bash
git push origin main
```
GitHub Actions auto-deploys on push to main branch.

### Re-indexing Documents
```bash
curl -X POST https://your-app-name.onrender.com/api/v1/reindex
```

---

## Security Best Practices

1. ✅ Never commit `.env` files
2. ✅ Use environment variables for all secrets
3. ✅ Enable rate limiting (already configured)
4. ✅ Use HTTPS only (Render provides this)
5. ✅ Keep dependencies updated
6. ✅ Monitor API usage and costs

---

## Support

If you encounter issues:
1. Check Render logs
2. Check GitHub Actions logs
3. Review browser console errors
4. Refer to documentation:
   - Render: https://render.com/docs
   - Docusaurus: https://docusaurus.io/docs
   - FastAPI: https://fastapi.tiangolo.com/

---

## Next Steps

After successful deployment:
- [ ] Set up monitoring with UptimeRobot
- [ ] Configure custom domain (optional)
- [ ] Add analytics (Google Analytics, Plausible)
- [ ] Set up automated backups for Qdrant data
- [ ] Implement usage tracking and rate limit adjustments
