"""Small interpreted Python policy subset. No exec/eval or Python object access.

Not a general Python sandbox. Only listed syntax and sdk calls are supported.
Host adapters remain trusted and must implement deadlines and physical interlocks.
"""
from __future__ import annotations
import ast
import operator
from .contracts import canonical

_CALLS = {"observe", "perform", "verify"}
_NODES = (ast.Module, ast.FunctionDef, ast.arguments, ast.arg, ast.Return,
          ast.Assign, ast.Expr, ast.If, ast.Name, ast.Load, ast.Store,
          ast.Constant, ast.Dict, ast.List, ast.Tuple, ast.Subscript,
          ast.Call, ast.Attribute, ast.keyword, ast.Compare, ast.Eq, ast.NotEq,
          ast.Lt, ast.LtE, ast.Gt, ast.GtE, ast.In, ast.NotIn,
          ast.UnaryOp, ast.Not, ast.BoolOp, ast.And, ast.Or)
_OPS = {ast.Eq: operator.eq, ast.NotEq: operator.ne, ast.Lt: operator.lt,
        ast.LtE: operator.le, ast.Gt: operator.gt, ast.GtE: operator.ge,
        ast.In: lambda a,b: a in b, ast.NotIn: lambda a,b: a not in b}


def validate(source: str) -> ast.FunctionDef:
    if len(source.encode()) > 65536:
        raise ValueError("Policy exceeds 64 KiB")
    tree = ast.parse(source)
    if len(tree.body) != 1 or not isinstance(tree.body[0], ast.FunctionDef):
        raise ValueError("Expected only def run_policy(sdk, params)")
    fn = tree.body[0]
    if (fn.name != "run_policy" or [a.arg for a in fn.args.args] != ["sdk", "params"]
            or fn.decorator_list or fn.args.defaults or fn.args.kwonlyargs
            or fn.args.vararg or fn.args.kwarg or fn.args.posonlyargs or fn.returns):
        raise ValueError("Invalid policy signature")
    nodes = list(ast.walk(tree))
    if len(nodes) > 4096:
        raise ValueError("Policy exceeds node budget")
    for node in nodes:
        if not isinstance(node, _NODES):
            raise ValueError(f"Unsupported policy syntax: {type(node).__name__}")
        if isinstance(node, ast.FunctionDef) and node is not fn:
            raise ValueError("Nested functions are not allowed")
        if isinstance(node, ast.Name) and node.id.startswith("_"):
            raise ValueError("Private identifiers are not allowed")
        if isinstance(node, ast.Assign):
            if len(node.targets) != 1 or not isinstance(node.targets[0], ast.Name) or node.targets[0].id in {"sdk", "params"}:
                raise ValueError("Only local name assignment is allowed")
        if isinstance(node, ast.Attribute):
            if not isinstance(node.value, ast.Name) or node.value.id != "sdk" or node.attr not in _CALLS:
                raise ValueError("Only explicit sdk methods are allowed")
        if isinstance(node, ast.Call) and (not isinstance(node.func, ast.Attribute) or any(k.arg is None for k in node.keywords)):
            raise ValueError("Only direct sdk calls are allowed")
        if isinstance(node, ast.Dict) and any(k is None for k in node.keys):
            raise ValueError("Dictionary unpacking is not allowed")
        if isinstance(node, ast.arg) and node.annotation:
            raise ValueError("Annotations are not allowed")
    return fn


def run(source: str, sdk, params: dict):
    fn = validate(source)
    canonical(params)
    env = {"params": params}
    budget = [4096]

    def expr(n):
        budget[0] -= 1
        if budget[0] < 0:
            raise ValueError("Policy evaluation budget exceeded")
        if isinstance(n, ast.Constant): return n.value
        if isinstance(n, ast.Name): return env[n.id]
        if isinstance(n, (ast.List, ast.Tuple)): return [expr(e) for e in n.elts]
        if isinstance(n, ast.Dict): return {expr(k): expr(v) for k,v in zip(n.keys,n.values)}
        if isinstance(n, ast.Subscript): return expr(n.value)[expr(n.slice)]
        if isinstance(n, ast.UnaryOp): return not expr(n.operand)
        if isinstance(n, ast.BoolOp):
            value = expr(n.values[0])
            for e in n.values[1:]:
                if isinstance(n.op, ast.And) and not value: return value
                if isinstance(n.op, ast.Or) and value: return value
                value = expr(e)
            return value
        if isinstance(n, ast.Compare):
            left = expr(n.left)
            for op,right_node in zip(n.ops,n.comparators):
                right = expr(right_node)
                if not _OPS[type(op)](left,right): return False
                left = right
            return True
        if isinstance(n, ast.Call):
            method = {"observe": sdk.observe, "perform": sdk.perform, "verify": sdk.verify}[n.func.attr]
            return method(*[expr(a) for a in n.args], **{k.arg:expr(k.value) for k in n.keywords})
        raise ValueError("Unsupported expression")

    def block(statements):
        for n in statements:
            if isinstance(n, ast.Return): return True, expr(n.value) if n.value else None
            if isinstance(n, ast.Assign): env[n.targets[0].id] = expr(n.value)
            elif isinstance(n, ast.Expr): expr(n.value)
            elif isinstance(n, ast.If):
                returned, value = block(n.body if expr(n.test) else n.orelse)
                if returned: return True,value
        return False,None
    return block(fn.body)[1]
