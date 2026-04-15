## 2024-05-15 - Graph Batch Retrieval Fix
**Learning:** When querying Neo4j for multiple IDs, using `UNWIND` Cypher clause for batch retrieval significantly prevents N+1 query problems and database round-trips.
**Action:** Always batch `fetch_memories` queries in graph stores instead of looping over individual queries, effectively reducing query load and improving overall performance by orders of magnitude for sequential retrievals.
