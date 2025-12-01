# Physical AI Textbook: Teaching Robotics with LLMs & Simulation

A comprehensive, open-source textbook for teaching **Physical AI and Humanoid Robotics** using modern tools: ROS 2, NVIDIA Isaac Sim, digital twins, and Vision Language Agents (VLA).

**Live Book**: https://ranisultana.github.io/physical-ai-textbook/  
**Backend API**: https://physical-ai-backend.fly.dev (demo mode)  
**Demo Video**: [Link to be added after recording 90-second demo]

---

## 📚 Course Structure

This textbook is organized into **4 core modules** + **hands-on labs** and a **capstone project**:

| Module | Topic | Duration |
|--------|-------|----------|
| **1** | Robotic Nervous System (ROS 2) | Weeks 1–3 |
| **2** | Digital Twin & Simulation (Gazebo/Isaac) | Weeks 4–6 |
| **3** | NVIDIA Isaac & Perception | Weeks 7–8 |
| **4** | Vision & Language Agents (VLA) | Weeks 9–10 |
| **Capstone** | Build Your Own Physical AI System | Weeks 11–12 |

**Full syllabus**: [Week-by-Week Schedule](./docs-source/week-by-week.md)

---

## 🚀 Quick Start

### 1. Clone & Setup

```bash
git clone https://github.com/RaniaSultana/physical-ai-textbook.git
cd physical-ai-textbook

# Install dependencies
yarn install

# Copy environment template (edit with your API keys)
cp .env.example .env
```

### 2. Configure API Keys

Edit `.env` with your keys:

```bash
# LLM APIs
CLAUDE_CODE_API_KEY=sk-ant-...
OPENAI_GEMINI_KEY=sk-...

# Optional: Neon Postgres + Qdrant Cloud
NEON_DB_URL=postgresql://...
QDRANT_URL=https://...
QDRANT_API_KEY=...
```

**Get free API keys:**
- Claude: https://console.anthropic.com
- OpenAI: https://platform.openai.com/api-keys
- Neon DB: https://neon.tech (free tier)
- Qdrant: https://cloud.qdrant.io (free tier)

### 3. Start Local Development

```bash
# Option A: Run web + backend together
yarn dev

# Option B: Run separately
yarn web:start          # http://localhost:3000
yarn backend:start      # http://localhost:8000
```

### 4. View the Textbook

- **Book**: http://localhost:3000
- **API Docs**: http://localhost:8000/docs (Swagger UI)
- **Chat Interface**: [Embedded in web site]

---

## 🛠️ Tech Stack

### Frontend
- **Docusaurus v2** — Static site generator for course content
- **React 18** — Interactive components (chat, demos)
- **Tailwind CSS** — Utility-first styling
- **Spec-Kit Plus** (optional) — Component library

### Backend
- **FastAPI** — High-performance Python API
- **Anthropic Claude 3.5 Sonnet** — LLM for task planning
- **OpenAI GPT-4o** — Alternative LLM with function calling
- **Qdrant** — Vector database for RAG (semantic search)

### Infrastructure
- **Docker** — Containerization
- **PostgreSQL** — User data (Neon serverless)
- **GitHub Actions** — CI/CD pipeline
- **GitHub Pages** — Textbook hosting
- **Fly.io / Railway** — Backend deployment

---

## 📖 API Reference

### Chat Endpoint

Send a message and get an AI response:

```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{
    "messages": [{"role": "user", "content": "What is ROS 2?"}],
    "model": "claude",
    "system_prompt": "You are a helpful robotics assistant."
  }'
```

**Response:**
```json
{
  "role": "assistant",
  "content": "ROS 2 is a middleware platform...",
  "model": "claude"
}
```

### Embeddings Endpoint

Generate vector embeddings for text:

```bash
curl -X POST http://localhost:8000/embed \
  -H "Content-Type: application/json" \
  -d '{"text": "ROS 2 publish-subscribe pattern"}'
```

**Response:**
```json
{
  "embedding": [0.123, -0.456, ...],
  "dimension": 1536
}
```

### RAG Search Endpoint

Search the textbook using semantic similarity:

```bash
curl -X POST http://localhost:8000/rag-search \
  -H "Content-Type: application/json" \
  -d '{"query": "How do digital twins help in robotics?", "top_k": 5}'
```

**Response:**
```json
{
  "query": "How do digital twins...",
  "results": [
    {
      "id": "doc_1",
      "text": "A digital twin is...",
      "score": 0.95,
      "metadata": {"module": "2", "section": "Introduction"}
    }
  ],
  "count": 1
}
```

### WebSocket Endpoint

Connect for real-time chat:

```bash
# Connect to ws://localhost:8000/ws
# Send: {"content": "Hello!"}
# Receive: {"status": "received", "message": "Hello!", "timestamp": "..."}
```

---

## 🤖 Agents

### Claude Agent

Task planning with Claude:

```python
from packages.agents import ClaudeAgent

agent = ClaudeAgent()
plan = agent.plan_task("Pick up the red cube from the table")
print(plan)
# Output: {"plan": [...], "reasoning": "...", "model": "claude-3-5-sonnet"}

results = agent.execute_plan(plan["plan"])
```

### OpenAI Agent

Function calling with GPT-4o:

```python
from packages.agents import OpenAIAgent

agent = OpenAIAgent()
response = agent.chat("Move the robot to position (0.3, 0.2, 0.5)")
print(response)

# If response contains function calls, execute them:
if response["has_function_call"]:
    func = response["response"].function_call
    result = agent.execute_function(func.name, json.loads(func.arguments))
```

---

## 🐳 Docker Setup

Run the full stack locally with Docker:

```bash
# Start all services (backend, postgres, qdrant, pgadmin)
docker-compose up -d

# Check services
docker-compose ps

# Backend API: http://localhost:8000
# PgAdmin: http://localhost:5050 (admin@admin.com / admin)
# Qdrant: http://localhost:6333

# Shutdown
docker-compose down
```

---

## 📹 Demo Video

### Record a 90-Second Demo

Follow these steps to create and share a demo:

1. **Setup**:
   ```bash
   yarn dev    # Start web + backend
   ```

2. **Script** (use `demo-script.sh` for automated walkthrough):
   ```bash
   chmod +x demo-script.sh
   ./demo-script.sh
   ```

3. **Record** (macOS):
   ```bash
   # Press Cmd+Shift+5 to start screen recording
   # Or use QuickTime Player > File > New Screen Recording
   ```

4. **Edit** (keep under 90 seconds):
   - Show textbook landing page (15s)
   - Chat with Claude about ROS 2 (30s)
   - RAG search example (20s)
   - Show GitHub repo structure (15s)

5. **Upload**:
   - YouTube: Unlisted video
   - GitHub Releases: Attach .mp4 file
   - Drive/Vimeo: Get shareable link

6. **Update README**:
   ```markdown
   **Demo Video**: [Your Link Here]
   ```

---

## 🚀 Deployment

### Deploy Textbook (GitHub Pages)

```bash
# Automatic: GitHub Actions on `git push main`
# Manual:
./infra/deploy-gh-pages.sh
```

**Result**: https://ranisultana.github.io/physical-ai-textbook/

### Deploy Backend

**Option 1: Fly.io**
```bash
# Install fly CLI
curl -L https://fly.io/install.sh | sh

# Deploy
./infra/deploy-backend.sh fly
```

**Option 2: Railway**
```bash
# Install railway CLI
npm i -g @railway/cli

# Deploy
./infra/deploy-backend.sh railway
```

**Option 3: Docker Registry**
```bash
./infra/deploy-backend.sh docker
```

---

## 📂 Project Structure

```
physical-ai-textbook/
├── apps/
│   └── web/
│       ├── src/
│       │   └── ChatBot.tsx          # Interactive chat component
│       ├── package.json
│       ├── spec-kit.config.js       # Optional Spec-Kit Plus config
│       └── build/                   # Static build output
├── packages/
│   ├── backend/
│   │   ├── main.py                  # FastAPI app
│   │   ├── Dockerfile
│   │   ├── requirements.txt
│   │   └── tests/
│   │       └── test_api.py
│   └── agents/
│       ├── claude_agent.py          # Claude tool use
│       ├── openai_agent.py          # OpenAI function calling
│       └── tests/
├── infra/
│   ├── deploy-gh-pages.sh
│   ├── deploy-backend.sh
│   └── setup-dev.sh
├── docs-source/
│   ├── index.md                     # Course intro
│   ├── module-1-ros2.md
│   ├── module-2-digital-twin.md
│   ├── module-3-isaac.md
│   ├── module-4-vla.md
│   ├── week-by-week.md
│   ├── hardware-lab.md
│   └── capstone.md
├── examples/
│   └── ros2/
│       └── hello_node.py            # ROS 2 example
├── .github/
│   └── workflows/
│       └── ci.yml                   # GitHub Actions CI/CD
├── docker-compose.yml
├── .env.example                     # Environment template
├── package.json                     # Root monorepo config
├── sidebars.js                      # Docusaurus navigation
└── README.md                        # This file
```

---

## 🧪 Testing

### Run All Tests

```bash
yarn test
```

### Run Specific Tests

```bash
# Backend API tests
yarn backend:test

# Agents tests
yarn agents:test

# Web build test
yarn web:build
```

---

## 🤝 Contributing

See [CONTRIBUTING.md](./CONTRIBUTING.md) for guidelines on:
- How to add new modules
- Submitting bug reports
- Requesting features
- Code style and best practices

**Quick contribution steps:**

1. Fork the repo
2. Create a branch: `git checkout -b feature/my-feature`
3. Make changes and test: `yarn test`
4. Commit: `git commit -m "feat: add my feature"`
5. Push: `git push origin feature/my-feature`
6. Open a Pull Request

---

## 📞 Support & Discussion

- **GitHub Issues**: Report bugs and request features
- **GitHub Discussions**: Ask questions and share ideas
- **Office Hours**: 2 hours/week (Tuesdays 2-4 PM UTC on Zoom)
- **Email**: rani@example.com (replace with actual)

---

## 📜 License

This project is licensed under the **MIT License** — see [LICENSE](./LICENSE) for details.

---

## 🎓 Course Credits

**Instructor**: Rani Sultana  
**Institution**: [Your University]  
**Course Code**: [COURSE_CODE]  
**Credits**: 4 units (Lecture 1.5 hrs/week + Lab 1.5 hrs/week)

---

## 📚 References & Acknowledgments

- [ROS 2 Official Documentation](https://docs.ros.org/en/iron/)
- [NVIDIA Isaac Sim](https://docs.nvidia.com/isaac/isaac_sim/)
- [Anthropic Claude API](https://docs.anthropic.com)
- [OpenAI API](https://platform.openai.com/docs)
- [Vision Language Models for Robotics](https://arxiv.org/abs/2212.07981)

---

**Happy learning! 🤖🚀**
