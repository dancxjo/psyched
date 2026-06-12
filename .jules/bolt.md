## 2024-04-14 - Prevent N+1 queries in Neo4j with batch retrieval and UNWIND
**Learning:** When querying Neo4j for multiple IDs sequentially inside a loop (like resolving graph metadata for vector search results), it creates an N+1 query problem that significantly degrades performance.
**Action:** When querying Neo4j for multiple IDs, use batch retrieval with the `UNWIND` Cypher clause (e.g., `UNWIND $ids AS id MATCH (n {id: id}) RETURN n`) to fetch all nodes in a single database round-trip, which substantially improves retrieval speed.
