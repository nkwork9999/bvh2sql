#!/bin/bash
set -e

echo "=== Building bvh2sql Extension ==="

echo ""
echo "Building DuckDB extension..."
make release

echo ""
echo "=== Build Complete ==="
echo ""
ls -lh build/release/extension/bvh2sql/bvh2sql.duckdb_extension
echo ""
echo "✓ Extension built successfully"
