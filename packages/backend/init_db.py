"""Initialize Neon Postgres database schema for RAG."""

import os
import psycopg2
from psycopg2 import sql
import logging

logger = logging.getLogger(__name__)


def init_database():
    """Create sources table in Neon Postgres."""
    db_url = os.getenv("NEON_DB_URL", "")
    
    if not db_url:
        logger.warning("NEON_DB_URL not set; skipping database initialization")
        return False
    
    try:
        # Connect to database
        conn = psycopg2.connect(db_url)
        cursor = conn.cursor()
        
        # Create sources table (include optional session metadata)
        create_table_sql = """
        CREATE TABLE IF NOT EXISTS sources (
            id TEXT PRIMARY KEY,
            title TEXT NOT NULL,
            text TEXT NOT NULL,
            session_id TEXT NULL,
            session BOOLEAN DEFAULT FALSE,
            created_at TIMESTAMPTZ DEFAULT CURRENT_TIMESTAMP
        );
        """
        
        cursor.execute(create_table_sql)
        conn.commit()
        
        logger.info("✓ Sources table initialized successfully")
        
        cursor.close()
        conn.close()
        
        return True
        
    except psycopg2.OperationalError as e:
        logger.error(f"Failed to connect to database: {e}")
        return False
    except Exception as e:
        logger.error(f"Failed to initialize database: {e}")
        return False


if __name__ == "__main__":
    init_database()
