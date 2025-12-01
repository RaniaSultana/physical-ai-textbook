"""Tests for agents package."""

import pytest
from agents.claude_agent import ClaudeAgent
from agents.openai_agent import OpenAIAgent

def test_claude_agent_initialization():
    """Test that Claude agent can be imported."""
    # Note: This test won't fully work without API key, but checks structure
    assert ClaudeAgent is not None

def test_openai_agent_initialization():
    """Test that OpenAI agent can be imported."""
    assert OpenAIAgent is not None
