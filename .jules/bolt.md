## 2024-05-19 - GraphStore Batch Retrieval Optimization
**Learning:** Performing a sequence of individual `fetch_memory` calls (N+1 queries) significantly impacted the performance of the `recall` method in `MemoryService`.
**Action:** Always prefer batch retrieval (using Cypher `UNWIND` in Neo4j or analogous methods for other databases) when resolving multiple entities by ID to eliminate round-trips to the database.
