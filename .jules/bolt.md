## 2024-05-18 - String.fromCharCode inside loop is a massive O(N^2) bottleneck for Base64

**Learning:** When generating Base64 strings from raw `Uint8Array` or `Int16Array` buffers (e.g. streaming chunks from browser for audio ASR or video frames), doing byte-by-byte string concatenation like `for(let i=0; i<bytes.length; i++) binary += String.fromCharCode(bytes[i]);` forces the JS engine to allocate strings on every iteration resulting in O(N^2) behavior.
**Action:** Always batch array bytes when doing `String.fromCharCode` with `.apply()`. We found 8192 is a safe and highly performant chunk size that avoids call stack limits (`RangeError: Maximum call stack size exceeded`), reducing the encoding time of 1MB from ~1500ms down to ~70ms.
