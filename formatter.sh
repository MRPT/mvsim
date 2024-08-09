# formatter.sh
find mvsim_node_src/ -iname *.h -o -iname *.hpp -o -iname *.cpp -o -iname *.c | xargs clang-format-14 -i
