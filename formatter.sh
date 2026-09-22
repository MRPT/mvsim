# formatter.sh
find modules/ mvsim_node_src/ examples_cpp/ mvsim-cli/ mvsim-dataset-gen/ tests/ -iname *.h -o -iname *.hpp -o -iname *.cpp -o -iname *.c | xargs clang-format-14 -i
