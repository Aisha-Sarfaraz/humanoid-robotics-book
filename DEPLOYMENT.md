# Deployment Guide - RAG Chatbot

This guide explains how to deploy the Humanoid Robotics Book website with the RAG chatbot to production using Vercel and GitHub Pages.

## Architecture

- **Frontend**: GitHub Pages (static hosting)
- **Backend**: Vercel (Python/FastAPI serverless)
- **Vector Database**: Qdrant Cloud (free tier)
- **LLM**: Gemini API (primary) + OpenAI API (fallback)

---

## Step 1: Deploy Backend to Vercel

### 1.1 Install Vercel CLI (Optional)

```bash
npm install -g vercel
```

### 1.2 Deploy via Vercel Dashboard

1. **Go to Vercel**: https://vercel.com/
2. **Sign up** with your GitHub account
3. **Import Project**:
   - Click **"Add New..."** → **"Project"**
   - Import your repository: `humanoid-robotics-book`
   - Vercel will auto-detect the `vercel.json` configuration

4. **Configure Project**:
   - **Framework Preset**: Other
   - **Root Directory**: `./` (leave as is)
   - **Build Command**: Leave empty (serverless function)
   - **Output Directory**: Leave empty
   - **Install Command**: `pip install -r backend/requirements.txt`

### 1.3 Configure Environment Variables

In the Vercel project settings, add these environment variables:

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

### 1.4 Deploy

1. Click **"Deploy"**
2. Vercel will:
   - Install dependencies from `backend/requirements.txt`
   - Deploy the FastAPI app as serverless functions
   - Assign a URL like: `https://your-project.vercel.app`

3. Wait for deployment to complete (~2-5 minutes)
4. Test the health endpoint:
   ```
   https://your-project.vercel.app/api/v1/health
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
BACKEND_URL=https://your-project.vercel.app
```

### 3.2 Add GitHub Secret

1. Go to your GitHub repository
2. **Settings** → **Secrets and variables** → **Actions**
3. Click **"New repository secret"**
4. Add:
   - Name: `BACKEND_URL`
   - Value: `https://your-project.vercel.app`

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

### 5.1 Check Backend (Vercel)

1. Visit: `https://your-project.vercel.app/api/v1/health`
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

## Vercel Deployment Tips

### Serverless Function Limits

Vercel Free Tier:
- **Execution Time**: 10 seconds max per request
- **Memory**: 1024 MB
- **Bandwidth**: 100 GB/month
- **Invocations**: Unlimited

**Note**: Streaming responses work within these limits.

### Environment Variables

- Set in Vercel dashboard: **Project Settings** → **Environment Variables**
- Can set different values for Production, Preview, and Development
- Changes require redeployment

### Logs and Monitoring

- View real-time logs: Vercel Dashboard → **Deployments** → **Function Logs**
- Monitor usage: Vercel Dashboard → **Analytics**

### Custom Domain (Optional)

1. Go to Vercel project settings
2. **Domains** → **Add Domain**
3. Follow DNS configuration instructions

---

## Troubleshooting

### Backend Not Starting on Vercel

**Issue**: Function timeout or build errors

**Solutions**:
- Check Vercel deployment logs
- Verify all environment variables are set
- Ensure `backend/requirements.txt` has all dependencies
- Python version is compatible (3.9+)

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
- Verify backend URL is HTTPS (Vercel provides this)
- Check backend logs for errors
- Ensure streaming endpoint is working: `/api/v1/chat/stream`

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
- **Vercel**: Free tier (100 GB bandwidth/month)
- **Qdrant Cloud**: 1GB storage free
- **Neon Serverless**: 0.5GB storage, 3GB data transfer free
- **Gemini API**: 15 RPM free tier

**Total Cost**: $0/month with free tiers

### Paid Upgrades (if needed):

- **Vercel Pro**: $20/month (more bandwidth, priority support)
- **Qdrant**: $25/month for 2GB+
- **Neon**: $19/month for higher limits
- **Gemini API**: Pay-as-you-go after free tier

---

## Monitoring

### Vercel Dashboard

- Monitor function invocations
- View error logs
- Check bandwidth usage
- Analytics for API calls

### GitHub Actions

- Monitor deployment status
- View build logs
- Check deployment history

### Uptime Monitoring (Optional)

- Use https://uptimerobot.com/ (free)
- Monitor backend health endpoint
- Get alerts if service goes down

---

## Updating the Application

### Updating Backend

```bash
# Make changes to backend code
git add backend/
git commit -m "fix: update backend logic"
git push origin main
```

Vercel auto-deploys on push to main branch.

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
curl -X POST https://your-project.vercel.app/api/v1/reindex
```

Or use the API endpoint with your frontend.

---

## Security Best Practices

1. ✅ Never commit `.env` files
2. ✅ Use environment variables for all secrets
3. ✅ Enable rate limiting (already configured)
4. ✅ Use HTTPS only (Vercel provides this automatically)
5. ✅ Keep dependencies updated
6. ✅ Monitor API usage and costs
7. ✅ Set up Vercel environment variables for Production only

---

## Next Steps After Deployment

- [ ] Set up monitoring with UptimeRobot
- [ ] Configure custom domain (optional)
- [ ] Add analytics (Google Analytics, Plausible)
- [ ] Set up automated backups for Qdrant data
- [ ] Implement usage tracking and rate limit adjustments
- [ ] Configure Vercel Edge Functions for improved performance (optional)

---

## Support

If you encounter issues:

1. Check Vercel deployment logs
2. Check GitHub Actions logs
3. Review browser console errors
4. Refer to documentation:
   - Vercel: https://vercel.com/docs
   - Docusaurus: https://docusaurus.io/docs
   - FastAPI: https://fastapi.tiangolo.com/

---

## Quick Start Checklist

- [ ] Deploy backend to Vercel
- [ ] Add environment variables in Vercel
- [ ] Get backend URL from Vercel
- [ ] Add `BACKEND_URL` to GitHub Secrets
- [ ] Push to main branch (triggers auto-deploy)
- [ ] Test chatbot on GitHub Pages

**Deployment Complete! 🎉**
