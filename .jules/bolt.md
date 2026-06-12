## 2024-05-10 - Solved N+1 query problem for Neo4j memory retrieval
**Learning:** Memory retrieval via `MemoryService.recall` originally suffered from an N+1 query problem when hydrating vector results with graph metadata (fetching metadata one-by-one from Neo4j).
**Action:** When querying Neo4j, use a batch retrieval with `UNWIND` (e.g. `fetch_memories(memory_ids)`) to drastically reduce database round-trips and improve recall performance.
