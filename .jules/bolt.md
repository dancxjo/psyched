## 2026-04-22 - Batch Neo4j Graph Queries with UNWIND
**Learning:** Calling `fetch_memory` inside a loop for each memory ID during a vector store recall results in an N+1 query problem, causing significant latency overhead due to repeated database round-trips.
**Action:** Use batch retrieval with the `UNWIND` Cypher clause in Neo4j to fetch multiple records in a single round-trip, preventing N+1 query problems and significantly reducing database overhead.
