# Contributing to Physical AI Textbook

Thank you for your interest in contributing! This document provides guidelines for contributing to the project.

## Code of Conduct

Be respectful, inclusive, and professional. All contributors are expected to uphold this code.

## How to Contribute

### 1. Report Bugs

Found a bug? Open a GitHub Issue with:
- **Title**: Clear, descriptive summary
- **Description**: What happened and what you expected
- **Steps to reproduce**: How to trigger the bug
- **Environment**: OS, Python version, Node version, etc.

**Example:**
```
Title: Backend chat endpoint returns 500 on empty messages

Description:
Sending an empty message list to /chat causes a 500 error.

Steps:
1. POST to http://localhost:8000/chat
2. Body: {"messages": [], "model": "claude"}

Expected: 400 Bad Request
Actual: 500 Internal Server Error

Environment:
- Ubuntu 22.04
- Python 3.11.5
- FastAPI 0.104.1
```

### 2. Suggest Enhancements

Have an idea? Open an issue with:
- **Title**: What would you like to add?
- **Motivation**: Why is this useful?
- **Proposed solution**: How would it work?
- **Alternatives**: Any other approaches?

### 3. Add New Content (Modules, Exercises)

To add a new module or exercise:

1. **Create a markdown file** in `docs-source/`:
   ```bash
   touch docs-source/module-5-advanced-topic.md
   ```

2. **Add frontmatter** (title, description, tags):
   ```markdown
   ---
   title: "Module 5: Advanced Topic"
   description: "Learn advanced techniques..."
   tags: [advanced, robotics]
   ---
   ```

3. **Structure the content**:
   ```markdown
   # Module 5: Advanced Topic
   
   ## Learning Objectives
   - Objective 1
   - Objective 2
   
   ## Key Concepts
   ### Concept A
   Explanation...
   
   ## Reading
   - [Reference 1](link)
   - [Reference 2](link)
   
   ## Hands-on Exercises
   ### Exercise 5.1: ...
   ...
   
   ## Code Example
   \`\`\`python
   # Code here
   \`\`\`
   ```

4. **Update sidebars** in `sidebars.js`:
   ```javascript
   {
     type: 'doc',
     id: 'module-5-advanced-topic',
     label: 'Module 5: Advanced Topic',
   }
   ```

5. **Test locally**:
   ```bash
   yarn web:build
   yarn web:start
   # Check http://localhost:3000
   ```

6. **Submit a Pull Request**

### 4. Improve Code (Backend, Agents, Frontend)

To fix code or add features:

1. **Fork the repo** and create a feature branch:
   ```bash
   git checkout -b feature/my-feature
   ```

2. **Make changes** following code style:
   - **Python**: PEP 8 (use `black` for formatting)
   - **TypeScript/React**: ESLint + Prettier
   - **Commit messages**: Use conventional commits (feat:, fix:, docs:, etc.)

3. **Add tests**:
   ```bash
   # Backend
   touch packages/backend/tests/test_my_feature.py
   
   # Run tests
   yarn backend:test
   ```

4. **Test locally**:
   ```bash
   yarn test        # Run all tests
   yarn web:build   # Build web
   yarn backend:start  # Start backend
   ```

5. **Submit a Pull Request** with:
   - Description of changes
   - Link to related issue (if any)
   - Screenshots (if UI changes)
   - Test results

### 5. Fix the Documentation

Found a typo or unclear section? You can edit directly on GitHub:

1. Navigate to the file (e.g., `docs-source/module-1-ros2.md`)
2. Click the pencil icon (Edit)
3. Make your changes
4. Commit with message: `docs: fix typo in Module 1`

Or fork and submit a PR.

## Development Workflow

### Setup Development Environment

```bash
# Clone repo
git clone https://github.com/RaniaSultana/physical-ai-textbook.git
cd physical-ai-textbook

# Run setup script
./infra/setup-dev.sh

# Edit .env with your API keys
nano .env
```

### Local Testing

```bash
# Start everything
yarn dev

# Or run separately:
yarn web:start          # http://localhost:3000
yarn backend:start      # http://localhost:8000

# Run tests
yarn test
```

### Git Workflow

```bash
# Create a feature branch
git checkout -b feature/add-new-module

# Make changes
# ... edit files ...

# Add and commit
git add .
git commit -m "feat: add new module on advanced robotics"

# Push to your fork
git push origin feature/add-new-module

# Create a Pull Request on GitHub
```

## Commit Message Format

Use [Conventional Commits](https://www.conventionalcommits.org/):

```
type(scope): subject

body

footer
```

**Types:**
- `feat:` — New feature
- `fix:` — Bug fix
- `docs:` — Documentation
- `style:` — Code style (formatting, missing semicolons, etc.)
- `refactor:` — Code refactoring
- `test:` — Adding or updating tests
- `chore:` — Maintenance (dependencies, build, etc.)

**Examples:**
```
feat(backend): add embeddings endpoint
fix(chatbot): handle empty message gracefully
docs: update API reference in README
test(agents): add unit tests for Claude agent
```

## Code Style Guidelines

### Python (Backend)

Use [PEP 8](https://www.python.org/dev/peps/pep-0008/):

```python
# Good
def calculate_distance(point1: tuple, point2: tuple) -> float:
    """Calculate Euclidean distance between two points."""
    x1, y1 = point1
    x2, y2 = point2
    return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5

# Format with black
black packages/backend/

# Check with pylint
pylint packages/backend/main.py
```

### TypeScript/React (Frontend)

Follow [Google TypeScript Style Guide](https://google.github.io/styleguide/tsconfig.json):

```typescript
// Good
interface ChatMessage {
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
}

// Format with Prettier
prettier --write apps/web/src/

// Check with ESLint
eslint apps/web/src/
```

## Pull Request Process

1. **Update dependencies**: If you change `requirements.txt` or `package.json`, include updated lock files
2. **Update documentation**: If changing behavior, update relevant `.md` files
3. **Test thoroughly**: Run `yarn test` and verify no new warnings
4. **Keep it focused**: One feature per PR
5. **Add meaningful PR description**: Include what changed and why

**PR Template:**

```markdown
## Description
Brief description of changes

## Type of Change
- [ ] Bug fix
- [ ] New feature
- [ ] Documentation update
- [ ] Code refactoring

## Related Issue
Closes #123 (if applicable)

## Testing
- [ ] Unit tests added/updated
- [ ] Manual testing done
- [ ] No regressions found

## Screenshots (if UI change)
[Add screenshots here]

## Checklist
- [ ] Code follows style guidelines
- [ ] Self-review completed
- [ ] Comments added for complex logic
- [ ] Documentation updated
- [ ] No new warnings generated
```

## Review Process

1. At least one maintainer review required
2. All CI/CD checks must pass
3. No merge conflicts
4. Squash and merge to `main`

## Questions?

- Open a GitHub Discussion
- Email: rani@example.com (replace with actual)
- Join office hours (Tuesdays 2-4 PM UTC)

---

**Thank you for contributing! 🎉**
