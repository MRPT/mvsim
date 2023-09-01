.. _world_xml_parser:

World XML files parser
-----------------------------

While reading XML tag attributes or XML node values, MVSim XML reader
parses the following special expressions:

Variables
============

You can define XML-level "variables" and dereference their values later on using the 
notation ``${var_name}``. Using undefined variables is an error.
Example:

.. code-block:: xml

    <variable name="POS_X" value="10"></variable>
    
    ... x="${POS_X}" ...

.. note::
    ``<variable>`` tags can be defined only as children of the top-level ``<world>`` tag at the global scope.

Special variables
====================

- ``${MVSIM_CURRENT_FILE_DIRECTORY}`` is resolved to the base directory of the current file being parsed.
  It is evaluated on the fly as the file is read, so it can be used within included files (see: :ref:`world-includes`)
  to refer to their physical path.

Environment variables
========================

Environment variables can be dereferenced with the notation ``${var_name}``.
Using undefined variables is an error.

Output of an external program
===============================

The ``stdout`` result of an external tool can be invoked like ``$(cmd_to_run arg1 arg2 arg3...)``.

Mathematical expressions
===============================

Using the notation ``$f{...}``, arbitrary expressions can be evaluated using 
`MRPT-wrapped version <https://docs.mrpt.org/reference/latest/class_mrpt_expr_CRuntimeCompiledExpression.html>`_ of ``ExprTk``, 
which provide much more than just mathematical operations, but also ``if`` structures, etc.

.. code-block:: xml

    <variable name="POS_X" value="10"></variable>
    <variable name="R" value="2.0"></variable>
    
    ... x="$f{POS_X + R*cos(45*M_PI/180)}" ...


.. note:: By defining the environment variable ``MVSIM_VERBOSE_PARSE`` to ``1``,
    you can see all the details about the parser evaluations, useful for debugging.