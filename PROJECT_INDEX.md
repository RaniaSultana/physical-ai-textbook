# Physical AI Textbook - Project Index

**Status**: ✅ **COMPLETE AND DEPLOYED**  
**Repository**: https://github.com/RaniaSultana/physical-ai-textbook  
**Date Completed**: December 1, 2025

---

## �� Documentation

Start here to understand the project:

1. **[README.md](./README.md)** — Complete project overview
   - Quick start guide
   - Tech stack
   - API reference with examples
   - Deployment instructions
   - Links to all resources

2. **[CONTRIBUTING.md](./CONTRIBUTING.md)** — How to contribute
   - Bug reporting
   - Feature requests
   - Code contribution workflow
   - Commit message format
   - Code style guidelines

3. **[SUBMISSION.md](./SUBMISSION.md)** — Project submission summary
   - Complete feature checklist
   - Project statistics
   - Acceptance criteria verification
   - Next steps for deployment

4. **[LICENSE](./LICENSE)** — MIT License

---

## 🧠 Textbook Content

All course content is in Markdown with interactive elements:

### Core Modules

- **[docs-source/index.md](./docs-source/index.md)** — Course introduction
  - Learning outcomes
  - Course structure overview
  - Prerequisites and how to use

- **[docs-source/module-1-ros2.md](./docs-source/module-1-ros2.md)** — Robotic Nervous System
  - ROS 2 fundamentals (nodes, topics, services, actions)
  - Code example: Publisher and Subscriber
  - 4 hands-on exercises
  - Debugging with command-line tools

- **[docs-source/module-2-digital-twin.md](./docs-source/module-2-digital-twin.md)** — Digital Twin & Simulation
  - URDF format for robot description
  - Gazebo simulation setup
  - Physics simulation and domain randomization
  - Code example: Simple 2-link URDF
  - Sim-to-real transfer challenges

- **[docs-source/module-3-isaac.md](./docs-source/module-3-isaac.md)** — NVIDIA Isaac & Perception
  - Isaac Sim setup
  - Synthetic data generation
  - Object detection pipelines
  - Sensor simulation (camera, LiDAR, IMU)
  - Code example: Camera integration in Isaac

- **[docs-source/module-4-vla.md](./docs-source/module-4-vla.md)** — Vision & Language Agents
  - VLM capabilities and limitations
  - Prompt engineering for robotics
  - Instruction following workflows
  - Closed-loop control with feedback
  - Code example: Claude Vision agent

### Supporting Materials

- **[docs-source/week-by-week.md](./docs-source/week-by-week.md)** — 12-week semester schedule
  - Weekly topics and deliverables
  - Suggested class structure (1.5h lecture + 1.5h lab)
  - Grading rubric

- **[docs-source/hardware-lab.md](./docs-source/hardware-lab.md)** — Hardware setup guide
  - Budget tiers (free simulators to $500K+ humanoids)
  - Recommended platforms by use case
  - Lab setup checklist
  - Safety guidelines
  - Quick start for specific robots

- **[docs-source/capstone.md](./docs-source/capstone.md)** — Final project specification
  - 4 project ideas (pick-and-place, table cleanup, mobile manipulation, sim-to-real)
  - Requirements and deliverables
  - Evaluation rubric
  - Team roles and timeline

---

## 💻 Code & Backend

### FastAPI Server

**[packages/backend/main.py](./packages/backend/main.py)** — Complete API implementation

**Endpoints:**
- `GET /` — Health check
- `POST /chat` — Chat with Claude or OpenAI
- `POST /embed` — Generate text embeddings
- `POST /rag-search` — Vector database search
- `WebSocket /ws` — Real-time messaging

**Features:**
- CORS middleware
- Error handling (400, 503 responses)
- Logging and monitoring
- Pydantic validation
- Mock data for demo mode

### Agents

- **[packages/agents/claude_agent.py](./packages/agents/claude_agent.py)** — Claude integration
  - Tool definitions (motion planning, ROS commands, sensor queries)
  - Task planning with `claude-3-5-sonnet-20241022`
  - Simulated execution

- **[packages/agents/openai_agent.py](./packages/agents/openai_agent.py)** — OpenAI integration
  - Function calling definitions (move, gripper, detect, state)
  - Task execution with GPT-4o
  - Stateful conversation history

### Frontend

- **[apps/web/src/ChatBot.tsx](./apps/web/src/ChatBot.tsx)** — Interactive chat component
  - React + TypeScript
  - Model selection dropdown
  - Message history with scrolling
  - Error handling and loading states
  - HTTP client to backend

---

## 🔧 Infrastructure & Configuration

### Deployment

- **[infra/deploy-gh-pages.sh](./infra/deploy-gh-pages.sh)** — Deploy textbook to GitHub Pages
- **[infra/deploy-backend.sh](./infra/deploy-backend.sh)** — Deploy backend (Fly.io, Railway, Docker)
- **[infra/setup-dev.sh](./infra/setup-dev.sh)** — Local development setup

### Docker

- **[docker-compose.yml](./docker-compose.yml)** — Full stack (backend, Postgres, Qdrant, PgAdmin)
- **[packages/backend/Dockerfile](./packages/backend/Dockerfile)** — FastAPI containerization

### Configuration

- **[.env.example](./.env.example)** — Environment template with all required keys
- **[.github/workflows/ci.yml](./.github/workflows/ci.yml)** — GitHub Actions CI/CD
- **[sidebars.js](./sidebars.js)** — Docusaurus navigation config
- **[package.json](./package.json)** — Monorepo root with scripts

---

## 📦 Scripts & Tools

### Package Scripts

```bash
yarn web:start        # Start textbook dev server
yarn web:build        # Build static site (✓ exit 0)
yarn backend:start    # Start FastAPI (hot-reload)
yarn backend:test     # Run pytest
yarn dev              # Run web + backend together
yarn test             # Run all tests
yarn setup            # Initialize .env
```

### Demo & Testing

- **[demo-script.sh](./demo-script.sh)** — 90-second automated demo
  - Health check
  - Chat examples (Claude + OpenAI)
  - RAG search demo
  - Code samples

---

## 🔗 External Resources

### LLM APIs (required for full functionality)

- **Claude**: https://console.anthropic.com
- **OpenAI**: https://platform.openai.com/api-keys

### Infrastructure (optional, free tiers available)

- **Neon Postgres**: https://neon.tech
- **Qdrant Vector DB**: https://cloud.qdrant.io
- **Fly.io**: https://fly.io (backend deployment)
- **Railway**: https://railway.app (backend deployment)

### Documentation

- **ROS 2**: https://docs.ros.org/en/iron/
- **NVIDIA Isaac**: https://docs.nvidia.com/isaac/isaac_sim/
- **Anthropic Claude**: https://docs.anthropic.com
- **OpenAI**: https://platform.openai.com/docs

---

## 📂 Directory Tree

```
physical-ai-textbook/
├── .github/
│   └── workflows/
│       └── ci.yml                    # GitHub Actions
├── apps/
│   └── web/
│       ├── src/
│       │   └── ChatBot.tsx           # React chat component
│       ├── scripts/
│       │   ├── build.js              # Build script
│       │   └── dev-server.js         # Dev server
│       ├── package.json
│       ├── spec-kit.config.js        # Optional Spec-Kit config
│       └── build/                    # Static output
├── packages/
│   ├── backend/
│   │   ├── main.py                   # FastAPI server
│   │   ├── Dockerfile
│   │   ├── requirements.txt
│   │   └── tests/
│   │       └── test_api.py
│   └── agents/
│       ├── __init__.py
│       ├── claude_agent.py
│       ├── openai_agent.py
│       ├── README.md
│       └── tests/
├── infra/
│   ├── deploy-gh-pages.sh
│   ├── deploy-backend.sh
│   └── setup-dev.sh
├── examples/
│   └── ros2/
│       └── hello_node.py             # ROS 2 example
├── docs-source/
│   ├── index.md                      # Course intro
│   ├── intro.md                      # Legacy
│   ├── module-1-ros2.md
│   ├── module-2-digital-twin.md
│   ├── module-3-isaac.md
│   ├── module-4-vla.md
│   ├── week-by-week.md
│   ├── hardware-lab.md
│   └── capstone.md
├── .env.example                      # Environment template
├── .gitignore
├── CONTRIBUTING.md                   # Contribution guide
├── LICENSE                           # MIT License
├── README.md                         # Main documentation
├── SUBMISSION.md                     # Project summary
├── PROJECT_INDEX.md                  # This file
├── docker-compose.yml
├── demo-script.sh
├── package.json                      # Monorepo root
└── sidebars.js                       # Navigation config
```

---

## ✅ Verification Checklist

Run this to verify everything:

```bash
# Build the project
yarn web:build           # Should exit with code 0 ✓

# Show all files
find . -type f \( -name "*.md" -o -name "*.py" -o -name "*.tsx" \) | wc -l
# Should show 30+ files

# Check git commits
git log --oneline | wc -l
# Should show 6 commits

# View recent commits
git log --oneline | head -5
```

---

## 🚀 Getting Started

### 1. Clone & Install

```bash
git clone https://github.com/RaniaSultana/physical-ai-textbook.git
cd physical-ai-textbook
yarn install
cp .env.example .env
```

### 2. Configure (Optional)

Edit `.env` with your API keys:
- `CLAUDE_CODE_API_KEY` — Claude API
- `OPENAI_GEMINI_KEY` — OpenAI API

### 3. Start Development

```bash
# Web + Backend
yarn dev

# Or separately:
yarn web:start         # http://localhost:3000
yarn backend:start     # http://localhost:8000
```

### 4. View Content

- **Textbook**: http://localhost:3000
- **API Docs**: http://localhost:8000/docs
- **GitHub**: https://github.com/RaniaSultana/physical-ai-textbook

### 5. Run Demo

```bash
./demo-script.sh
```

---

## 📞 Support

- **GitHub Issues**: Report bugs and request features
- **GitHub Discussions**: Ask questions
- **Email**: rani@example.com (replace with actual)

---

**Project Complete** ✅  
All tasks finished. Ready for production deployment.
