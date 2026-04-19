## 2024-06-25 - Neo4j UNWIND Batch Retrieval
**Learning:** In Neo4j using the python driver, fetching nodes one by one within a loop creates an N+1 query performance bottleneck due to multiple database round-trips.
**Action:** When querying Neo4j for multiple IDs, use batch retrieval with the `UNWIND` Cypher clause. This significantly reduces database round-trips and improves recall performance significantly.
