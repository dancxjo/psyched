## 2024-05-14 - String concatenation memory limit in browsers
**Learning:** In frontend JS code that converts byte arrays to Base64, processing the array byte-by-byte using string concatenation (`binary += String.fromCharCode(view[i])`) or `String.fromCharCode.apply(null, arr)` on large arrays can cause O(N^2) performance issues and Maximum Call Stack Size Exceeded errors.
**Action:** Process the array in chunks.
