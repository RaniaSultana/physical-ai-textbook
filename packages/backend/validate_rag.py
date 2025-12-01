#!/usr/bin/env python3
"""
Validation script for RAG implementation.
Tests the code structure and syntax without requiring dependencies.
"""

import sys
import os
import ast


def validate_python_syntax(filepath):
    """Check if a Python file has valid syntax."""
    try:
        with open(filepath, 'r') as f:
            ast.parse(f.read())
        return True, None
    except SyntaxError as e:
        return False, str(e)


def validate_imports_exist(filepath):
    """Check if imports in file are defined."""
    try:
        with open(filepath, 'r') as f:
            tree = ast.parse(f.read())
        
        imports = []
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for alias in node.names:
                    imports.append(alias.name)
            elif isinstance(node, ast.ImportFrom):
                imports.append(node.module or '')
        
        return True, imports
    except Exception as e:
        return False, str(e)


def check_function_exists(filepath, func_name):
    """Check if a function is defined in file."""
    try:
        with open(filepath, 'r') as f:
            tree = ast.parse(f.read())
        
        for node in ast.walk(tree):
            if isinstance(node, ast.FunctionDef) and node.name == func_name:
                return True
        return False
    except Exception:
        return False


def check_class_exists(filepath, class_name):
    """Check if a class is defined in file."""
    try:
        with open(filepath, 'r') as f:
            tree = ast.parse(f.read())
        
        for node in ast.walk(tree):
            if isinstance(node, ast.ClassDef) and node.name == class_name:
                return True
        return False
    except Exception:
        return False


def main():
    """Run validation checks."""
    base_path = "/Users/apple/Documents/copilot-hackathon/physical-ai-textbook/packages/backend"
    
    print("=" * 60)
    print("RAG Implementation Validation")
    print("=" * 60)
    
    all_passed = True
    
    # Check main.py
    print("\n1. Validating main.py...")
    valid, err = validate_python_syntax(f"{base_path}/main.py")
    if valid:
        print("   ✓ Valid Python syntax")
    else:
        print(f"   ✗ Syntax error: {err}")
        all_passed = False
    
    # Check for new endpoints in main.py
    print("\n2. Checking for new endpoints in main.py...")
    endpoints = ["/ingest", "/query"]
    for endpoint in endpoints:
        with open(f"{base_path}/main.py", 'r') as f:
            content = f.read()
            if f'"{endpoint}"' in content or f"'{endpoint}'" in content:
                print(f"   ✓ Found {endpoint} endpoint")
            else:
                print(f"   ✗ Missing {endpoint} endpoint")
                all_passed = False
    
    # Check rag_service.py
    print("\n3. Validating rag_service.py...")
    valid, err = validate_python_syntax(f"{base_path}/rag_service.py")
    if valid:
        print("   ✓ Valid Python syntax")
    else:
        print(f"   ✗ Syntax error: {err}")
        all_passed = False
    
    print("\n4. Checking RAGService class...")
    if check_class_exists(f"{base_path}/rag_service.py", "RAGService"):
        print("   ✓ RAGService class exists")
    else:
        print("   ✗ RAGService class not found")
        all_passed = False
    
    print("\n5. Checking RAGService methods...")
    methods = ["ingest", "query", "_get_embedding"]
    for method in methods:
        if check_function_exists(f"{base_path}/rag_service.py", method):
            print(f"   ✓ Found {method} method")
        else:
            print(f"   ✗ Missing {method} method")
            all_passed = False
    
    # Check init_db.py
    print("\n6. Validating init_db.py...")
    valid, err = validate_python_syntax(f"{base_path}/init_db.py")
    if valid:
        print("   ✓ Valid Python syntax")
    else:
        print(f"   ✗ Syntax error: {err}")
        all_passed = False
    
    # Check test_rag.py
    print("\n7. Validating test_rag.py...")
    valid, err = validate_python_syntax(f"{base_path}/tests/test_rag.py")
    if valid:
        print("   ✓ Valid Python syntax")
    else:
        print(f"   ✗ Syntax error: {err}")
        all_passed = False
    
    print("\n8. Checking for test classes...")
    test_classes = ["TestRAGEndpoints", "TestRAGIntegration"]
    for test_class in test_classes:
        if check_class_exists(f"{base_path}/tests/test_rag.py", test_class):
            print(f"   ✓ Found {test_class} test class")
        else:
            print(f"   ✗ Missing {test_class} test class")
            all_passed = False
    
    # Check requirements.txt
    print("\n9. Checking requirements.txt...")
    with open(f"{base_path}/requirements.txt", 'r') as f:
        reqs = f.read()
        required_packages = ["psycopg2-binary", "httpx", "qdrant"]
        for pkg in required_packages:
            if pkg in reqs:
                print(f"   ✓ Found {pkg}")
            else:
                print(f"   ✗ Missing {pkg}")
                all_passed = False
    
    # Check for Postgres schema definition
    print("\n10. Checking Postgres schema...")
    with open(f"{base_path}/rag_service.py", 'r') as f:
        content = f.read()
        schema_elements = ["sources", "id TEXT PRIMARY KEY", "title TEXT", "created_at"]
        for elem in schema_elements:
            if elem in content:
                print(f"   ✓ Found schema element: {elem}")
            else:
                print(f"   ✗ Missing schema element: {elem}")
                all_passed = False
    
    # Check for Qdrant integration
    print("\n11. Checking Qdrant integration...")
    with open(f"{base_path}/rag_service.py", 'r') as f:
        content = f.read()
        qdrant_elements = ["QdrantClient", "VectorParams", "Distance.COSINE", "upsert"]
        for elem in qdrant_elements:
            if elem in content:
                print(f"   ✓ Found Qdrant element: {elem}")
            else:
                print(f"   ✗ Missing Qdrant element: {elem}")
                all_passed = False
    
    # Check for OpenAI embeddings
    print("\n12. Checking OpenAI embeddings...")
    with open(f"{base_path}/rag_service.py", 'r') as f:
        content = f.read()
        if "text-embedding-3-small" in content:
            print("   ✓ OpenAI embeddings model configured")
        else:
            print("   ✗ OpenAI embeddings model not found")
            all_passed = False
    
    print("\n" + "=" * 60)
    if all_passed:
        print("✅ ALL VALIDATION CHECKS PASSED")
        return 0
    else:
        print("❌ SOME VALIDATION CHECKS FAILED")
        return 1


if __name__ == "__main__":
    sys.exit(main())
