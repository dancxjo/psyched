## 2024-05-24 - Avoid O(N^2) byte-by-byte string concatenation in JavaScript
**Learning:** Using a `for` loop to concatenate bytes into a string one-by-one (`binary += String.fromCharCode(view[i])`) or using `apply` on a huge array `String.fromCharCode.apply(null, largeArray)` causes performance bottlenecks (O(N^2) time complexity or "Maximum call stack size exceeded").
**Action:** When converting byte arrays to strings for Base64 encoding, chunk the data into manageable sizes (e.g. 8192 bytes) and use `String.fromCharCode.apply(null, chunk)` to concatenate chunks.
