# ChatKit-Frontend Integration Skill

## Name
chatkit-frontend

## Description
Comprehensive skill for integrating OpenAI ChatKit.js into applications. Provides step-by-step guidance for building production-ready AI chat interfaces with response streaming, tool integration, and rich interactive widgets. ChatKit is a framework-agnostic, drop-in chat solution that eliminates the need to construct custom interfaces or manage complex chat state independently.

## Use Cases
- Building AI-powered chatbots and assistants
- Integrating conversational interfaces into documentation sites
- Creating customer support chat widgets
- Implementing RAG (Retrieval-Augmented Generation) chat experiences
- Building tutoring and educational chat systems
- Adding agentic reasoning visualization
- Developing conversational AI applications with OpenAI models

## Prerequisites
- Node.js 18+ installed
- React 18+ (for React integration) OR vanilla JavaScript
- OpenAI API key (for hosted backend) OR custom backend
- Basic understanding of TypeScript/JavaScript
- Familiarity with REST APIs and async/await patterns

## Framework Support
- **Primary**: React 18+ via `@openai/chatkit-react`
- **Web Component**: Vanilla JS via `openai-chatkit` element
- **Compatible**: Vue, Svelte, Angular (via web component)
- **Backend Options**:
  - OpenAI Agent Builder (hosted, managed infrastructure)
  - ChatKit Python SDK (self-hosted)
  - Custom backend implementation (FastAPI, Express, etc.)

## Core Capabilities

### 1. Deployment Models
- **OpenAI-Hosted**: Fastest setup with Agent Builder workflows managed by OpenAI
- **Self-Hosted**: Maximum control with custom backend implementation
- Flexible configuration options for both patterns
- Easy switching between deployment models

### 2. UI Features
- **Response streaming** for natural, gradual text display
- **Tool and workflow integration** with agentic reasoning visualization
- **Rich interactive widgets** rendered within chat context
- **File attachments and image uploads** with preview
- **Thread management** for organizing multiple conversations
- **Source annotations** and entity tagging for transparency
- **Dark/light theme support** with auto-detection
- **Mobile-responsive** design out-of-box

### 3. Customization
- Deep UI customization to match application branding
- Theme configuration (colors, typography, spacing, radius)
- Custom widgets and interactive components
- Composer (input area) customization with placeholders
- Header configuration (title, icon, actions)
- CSS variable overrides for fine-tuned control

### 4. Integration Patterns
- React hook-based integration with `useChatKit()`
- Web component drop-in for framework-agnostic use
- Custom backend integration with fetch override
- Authentication and authorization via custom headers
- Persistent conversation history with thread support
- Event listeners for message lifecycle

## Architecture Pattern

### OpenAI-Hosted Pattern
```
┌─────────────────────┐
│    React App        │
│  (ChatKit UI)       │
│  - useChatKit()     │
│  - <ChatKit />      │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│  Agent Builder      │ (OpenAI-managed)
│  (Backend)          │
│  - Workflows        │
│  - Model routing    │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│   OpenAI API        │
│  - GPT-4o           │
│  - GPT-4-turbo      │
└─────────────────────┘
```

### Self-Hosted Pattern
```
┌─────────────────────┐
│    React App        │
│  (ChatKit UI)       │
│  - Custom fetch()   │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│  Custom Backend     │ (Your infrastructure)
│  (FastAPI, Express) │
│  - Auth middleware  │
│  - Business logic   │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│   OpenAI API        │
│   or Custom LLM     │
│  (Anthropic, etc.)  │
└─────────────────────┘
```

## Standard File Structure

```
project/
├── src/
│   ├── components/
│   │   └── Chat/
│   │       ├── ChatKitWrapper.jsx     # Main ChatKit integration
│   │       ├── ChatKitConfig.js       # Configuration object
│   │       └── CustomWidgets.jsx      # Optional custom widgets
│   ├── hooks/
│   │   └── useChatKitSetup.js        # Custom ChatKit setup hook
│   ├── styles/
│   │   └── chatkit-theme.css          # Custom theme overrides
│   └── utils/
│       └── chatkit-helpers.js         # Utility functions
├── .env.local                          # Environment variables
└── package.json
```

## Configuration Templates

### React Integration (OpenAI-Hosted)
```javascript
import { useChatKit, ChatKit } from '@openai/chatkit-react';

function MyChat() {
  const { control } = useChatKit({
    api: {
      url: 'https://api.openai.com/v1/chatkit',
      domainKey: process.env.REACT_APP_OPENAI_DOMAIN_KEY
    },
    model: 'gpt-4o',
    theme: {
      mode: 'light',
      accent: '#4f6fef'
    },
    composer: {
      placeholder: 'Ask me anything...'
    },
    header: {
      title: 'AI Assistant',
      icon: '🤖'
    }
  });

  return (
    <ChatKit
      control={control}
      className="h-[600px] w-[400px] rounded-lg shadow-lg"
    />
  );
}

export default MyChat;
```

### React Integration (Self-Hosted Backend)
```javascript
import { useChatKit, ChatKit } from '@openai/chatkit-react';

function MyChat() {
  const { control } = useChatKit({
    api: {
      url: process.env.REACT_APP_CHAT_API || 'http://localhost:8000/api/v1/chat',
      fetch: async (endpoint, options) => {
        // Custom fetch with authentication
        const token = localStorage.getItem('authToken');

        return fetch(endpoint, {
          ...options,
          headers: {
            ...options.headers,
            'Authorization': `Bearer ${token}`,
            'Content-Type': 'application/json'
          },
          credentials: 'include'
        });
      }
    },
    theme: {
      mode: 'dark',
      accent: '#8B5CF6',
      font: {
        family: 'Inter, system-ui, sans-serif',
        size: '14px'
      }
    }
  });

  return <ChatKit control={control} className="h-full w-full" />;
}

export default MyChat;
```

### Vanilla JavaScript (Web Component)
```javascript
// Create and configure ChatKit web component
const chatkit = document.createElement('openai-chatkit');

chatkit.setOptions({
  api: {
    url: 'https://api.openai.com/v1/chatkit',
    domainKey: 'your-domain-key-here'
  },
  model: 'gpt-4o',
  theme: {
    mode: 'light',
    accent: '#4f6fef'
  },
  composer: {
    placeholder: 'Type your message...'
  }
});

// Style the component
chatkit.classList.add('h-[600px]', 'w-[320px]', 'rounded-lg');

// Add to DOM
document.getElementById('chat-container').appendChild(chatkit);

// Listen to events
chatkit.addEventListener('message', (event) => {
  console.log('New message:', event.detail);
});
```

### Docusaurus Integration
```javascript
// src/theme/Root.js
import React from 'react';
import ChatKitWrapper from '../components/ChatKitWrapper';

export default function Root({ children }) {
  return (
    <>
      {children}
      <ChatKitWrapper />
    </>
  );
}

// src/components/ChatKitWrapper.jsx
import React, { useState } from 'react';
import { useChatKit, ChatKit } from '@openai/chatkit-react';
import './ChatKitWrapper.css';

export default function ChatKitWrapper() {
  const [isOpen, setIsOpen] = useState(false);

  const { control } = useChatKit({
    api: {
      url: process.env.REACT_APP_CHAT_API
    },
    theme: {
      mode: 'light',
      accent: 'var(--ifm-color-primary)'
    }
  });

  return (
    <>
      <button
        onClick={() => setIsOpen(!isOpen)}
        className="chatkit-toggle"
        aria-label="Toggle chat"
      >
        💬
      </button>

      {isOpen && (
        <div className="chatkit-container">
          <ChatKit control={control} className="h-[500px] w-[350px]" />
        </div>
      )}
    </>
  );
}
```

### Theme Customization
```javascript
{
  theme: {
    mode: 'dark',              // 'light' | 'dark' | 'auto'
    accent: '#8B5CF6',         // Brand color
    font: {
      family: 'Inter, system-ui, sans-serif',
      size: '14px'
    },
    radius: '8px',             // Border radius
    grayscale: false           // Accessibility mode
  }
}
```

### Advanced Configuration with Tools
```javascript
{
  api: { /* ... */ },
  theme: { /* ... */ },
  tools: [
    {
      name: 'search_docs',
      description: 'Search documentation',
      parameters: {
        type: 'object',
        properties: {
          query: { type: 'string' }
        }
      }
    }
  ],
  widgets: {
    'citation': CitationWidget,
    'code-block': CodeBlockWidget
  },
  attachments: {
    enabled: true,
    maxSize: 10 * 1024 * 1024, // 10MB
    accept: 'image/*,.pdf,.doc,.docx'
  }
}
```

## Environment Variables

### OpenAI-Hosted
```env
# Required for OpenAI-hosted backend
REACT_APP_OPENAI_DOMAIN_KEY=your-domain-key-from-agent-builder
REACT_APP_OPENAI_API_KEY=sk-...

# Optional
REACT_APP_CHATKIT_MODEL=gpt-4o
```

### Self-Hosted
```env
# Required for custom backend
REACT_APP_CHAT_API=http://localhost:8000/api/v1/chat
REACT_APP_OPENAI_API_KEY=sk-...

# Optional auth
REACT_APP_AUTH_ENABLED=true
```

## Common Integration Scenarios

### Scenario 1: Add ChatKit to Existing React App
**Steps**:
1. Install package: `npm install @openai/chatkit-react`
2. Create ChatKit component with `useChatKit` hook
3. Configure API endpoint (hosted or self-hosted)
4. Customize theme to match app design system
5. Add component to desired page/route
6. Set up environment variables
7. Test message flow and streaming

**Use case**: Adding AI assistant to existing dashboard, admin panel, or customer portal.

### Scenario 2: Replace Custom Chat with ChatKit
**Steps**:
1. Analyze existing chat interface features and requirements
2. Map custom features to ChatKit configuration options
3. Migrate message history format to ChatKit structure
4. Update backend API to match ChatKit contract (if self-hosted)
5. Implement side-by-side testing with feature flag
6. Gradually phase out custom implementation
7. Remove legacy code after validation

**Use case**: Modernizing outdated chat UI with production-ready solution.

### Scenario 3: Integrate with Docusaurus (Documentation Site)
**Steps**:
1. Create ChatKit wrapper component in `src/components/`
2. Add to `src/theme/Root.js` for global availability
3. Configure self-hosted backend (FastAPI, Express)
4. Style to match Docusaurus theme variables
5. Handle authentication if site has protected content
6. Add toggle button for chat visibility
7. Test on mobile and desktop viewports

**Use case**: Adding interactive documentation assistant like this project.

### Scenario 4: Build Customer Support Widget
**Steps**:
1. Set up ChatKit with OpenAI-hosted backend
2. Configure Agent Builder workflow for support routing
3. Customize theme to match brand guidelines
4. Implement file attachment for screenshots
5. Add thread management for ticket tracking
6. Integrate with CRM/ticketing system webhooks
7. Monitor conversations and analytics

**Use case**: Embedded support chat on product website.

### Scenario 5: Create RAG Chatbot with Custom Data
**Steps**:
1. Build custom backend with vector database (Qdrant, Pinecone)
2. Implement embedding generation for documents
3. Set up retrieval pipeline for context injection
4. Configure ChatKit with self-hosted API endpoint
5. Add source citations using widgets
6. Implement semantic search tools
7. Deploy with authentication

**Use case**: Knowledge base chatbot for internal wiki or product docs.

## Validation Checklist

Before considering ChatKit integration complete, verify:

- [ ] **ChatKit component renders** without errors in browser
- [ ] **Messages send successfully** and appear in UI
- [ ] **Messages receive successfully** from backend
- [ ] **Response streaming works** (gradual text display, not all at once)
- [ ] **Theme matches application** design system colors and fonts
- [ ] **File attachments work** (upload, preview, send) if enabled
- [ ] **Thread management functional** (create, switch, delete threads)
- [ ] **Custom backend authenticated** properly with headers
- [ ] **Mobile responsive** on small screens (<768px)
- [ ] **Accessible** (keyboard navigation, screen reader compatible)
- [ ] **Error handling** for network failures shows user-friendly messages
- [ ] **Loading states** display during API calls
- [ ] **Environment variables** configured correctly
- [ ] **CORS configured** on backend to allow frontend origin
- [ ] **API costs monitored** if using OpenAI API

## Troubleshooting

### Issue: ChatKit component not rendering
**Symptoms**: Blank screen, no chat UI visible, console errors

**Solutions**:
1. Check console for errors - look for package import failures
2. Verify package installed: `npm list @openai/chatkit-react`
3. Ensure React version is 18+: `npm list react`
4. Clear build cache: `rm -rf node_modules/.cache && npm run build`
5. Check component is imported correctly
6. Verify JSX syntax is correct (self-closing tags, className not class)

### Issue: API calls failing
**Symptoms**: Messages don't send, network errors in console, 401/403/500 responses

**Solutions**:
1. Verify API URL is correct and accessible (test with curl)
2. Check CORS settings on backend allow your frontend origin
3. Validate API keys/domain keys are correct and not expired
4. Check network tab in dev tools for request/response details
5. Ensure `fetch` function includes proper headers
6. Test backend endpoint independently (Postman, curl)
7. Verify environment variables are loaded (`console.log(process.env)`)

### Issue: Theme not applying
**Symptoms**: ChatKit renders with default theme, custom colors not showing

**Solutions**:
1. Ensure theme object structure matches documentation
2. Check CSS variable inheritance from parent elements
3. Verify className prop is applied to ChatKit component
4. Inspect element to see computed styles
5. Check for CSS specificity conflicts
6. Try inline style overrides to diagnose
7. Ensure `mode` is set correctly ('light', 'dark', or 'auto')

### Issue: Messages not streaming
**Symptoms**: Messages appear all at once, no gradual typing effect

**Solutions**:
1. Backend must support Server-Sent Events (SSE) or streaming responses
2. Check Content-Type header is `text/event-stream`
3. Verify response format matches ChatKit expectations
4. Disable buffering on proxies/CDN (nginx, CloudFlare)
5. Test streaming with curl: `curl -N endpoint`
6. Check browser supports EventSource API
7. Review backend implementation for `yield` or streaming patterns

### Issue: File uploads not working
**Symptoms**: Upload button doesn't work, files don't attach, errors on upload

**Solutions**:
1. Verify `attachments.enabled: true` in configuration
2. Check file size under `maxSize` limit
3. Ensure file type matches `accept` pattern
4. Backend must handle multipart/form-data
5. Check CORS allows file upload methods
6. Verify backend has file storage configured
7. Test with small file first (< 1MB)

### Issue: Authentication errors
**Symptoms**: 401 Unauthorized, token invalid, session expired

**Solutions**:
1. Verify auth token is being sent in headers
2. Check token hasn't expired (decode JWT)
3. Ensure custom `fetch` function includes Authorization header
4. Validate token format matches backend expectations
5. Check backend validates tokens correctly
6. Refresh token if expired
7. Clear storage and re-authenticate

## Documentation Resources

- **ChatKit.js Official Docs**: https://openai.github.io/chatkit-js/
- **OpenAI ChatKit Guide**: https://platform.openai.com/docs/guides/chatkit
- **API Reference**: https://platform.openai.com/docs/api-reference/chatkit
- **Theming Guide**: https://platform.openai.com/docs/guides/chatkit-themes
- **Actions Guide**: https://platform.openai.com/docs/guides/chatkit-actions
- **Advanced Integrations**: https://platform.openai.com/docs/guides/custom-chatkit
- **Starter App**: https://github.com/openai/openai-chatkit-starter-app
- **Advanced Samples**: https://github.com/openai/openai-chatkit-advanced-samples

## Usage Instructions

To invoke this skill in Claude Code, use:

```
Help me integrate ChatKit into my React application
Add OpenAI ChatKit to my Docusaurus site
Replace my custom chat with ChatKit
Set up ChatKit with a custom FastAPI backend
Configure ChatKit theme to match my brand
Debug ChatKit streaming issues
```

## Version Compatibility

- **ChatKit.js**: Latest (check npm for current version)
- **React**: 18.0.0+ (React 18 required for concurrent features)
- **Node.js**: 18.0.0+ (for build tools and package management)
- **OpenAI API**: v1 (API version)
- **TypeScript**: 5.0.0+ (optional, but recommended for type safety)

## Notes

- **State Management**: ChatKit handles complex chat state management automatically - no need for Redux/Zustand
- **Response Streaming**: Requires backend that supports Server-Sent Events (SSE) or streaming responses
- **Custom Widgets**: Require understanding ChatKit widget API and React component patterns
- **Production Deployment**: Always use environment variables for API keys, never hardcode
- **Cost Management**: Monitor OpenAI API usage, implement rate limiting, set up billing alerts
- **Bundle Size**: ChatKit adds ~50KB gzipped to bundle - consider code splitting for large apps
- **Browser Support**: Works in all modern browsers (Chrome, Firefox, Safari, Edge), IE11 not supported
- **Accessibility**: ChatKit includes ARIA labels and keyboard navigation out-of-box
- **Localization**: Currently English only, custom translations require additional configuration
