#!/usr/bin/env python3
"""Arm-independent chess layer (LAPTOP). No arm motion: this turns chess moves into physical pick/place steps in
arm-frame millimetres, reads the board state from tagged pieces (find_blocks.py JSON), and detects the human's move.
Any arm that can "pick the block at (x, y), place it at (x, y)" can execute the steps (myCobot now, the bigger arm later).

Pieces = tagged blocks, one AprilTag 36h11 ID per piece (config/chess_pieces_*.json, IDs 100-139; IDs 1-4, 10-17, 30-34
are already used by the block/base/hand tags). A board is a JSON file with the arm-frame centre of every square, so it can
be a regular grid (--make-grid) or come from a program that DRAWS the board with the arm (then the drawn coordinates are
the board, with no board-to-arm calibration).

Usage (python = ~/venvs/realsense/bin/python):
  chess_core_20260926_200952.py --selftest
  chess_core_20260926_200952.py --make-grid OUT.json --a1 X Y --square 45 --file-dir-deg 90 --rank-dir-deg 0 \
      --graveyard X Y --graveyard-dir-deg 90 [--reserve-origin X Y]
  chess_core_20260926_200952.py --observe blocks.json --board BOARD.json      # FEN of a find_blocks.py snapshot
  chess_core_20260926_200952.py --play --board BOARD.json [--robot-color black]  # terminal game vs Stockfish; prints steps

Physical-step rules: a capture first moves the captured piece to the next free graveyard slot; castling moves the king,
then the rook; en passant removes the pawn beside the destination; a promotion moves the pawn to the graveyard and puts a
piece of the promoted type on the square (from the reserve, else from the graveyard, else asks the human).
"""
import argparse
import datetime
import json
import math
import os
import platform
import random
import sys

import chess
import chess.engine

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
STOCKFISH = os.path.expanduser("~/opt/stockfish/stockfish/stockfish-ubuntu-x86-64-avx2")
DEFAULT_PIECES = os.path.join(REPO, "config", "chess_pieces_20260926_200952.json")
SNAP_TOL = 0.35   # a piece counts as on a square when within SNAP_TOL x square size of its centre


# ---------------------------------------------------------------- board geometry
def make_grid(a1, square, file_dir_deg, rank_dir_deg, graveyard, graveyard_dir_deg, reserve_origin=None, per_row=8):
    """Regular board: square centres from the a1 centre, the a->h direction and the 1->8 direction (arm frame)."""
    fd = (math.cos(math.radians(file_dir_deg)), math.sin(math.radians(file_dir_deg)))
    rd = (math.cos(math.radians(rank_dir_deg)), math.sin(math.radians(rank_dir_deg)))
    centres = {}
    for sq in chess.SQUARES:
        f, r = chess.square_file(sq), chess.square_rank(sq)
        centres[chess.square_name(sq)] = [round(a1[0] + square * (f * fd[0] + r * rd[0]), 2),
                                          round(a1[1] + square * (f * fd[1] + r * rd[1]), 2)]
    gd = (math.cos(math.radians(graveyard_dir_deg)), math.sin(math.radians(graveyard_dir_deg)))
    # graveyard rows step away from the board along the rank direction's opposite side of the origin
    gy = [[round(graveyard[0] + square * ((k % per_row) * gd[0] - (k // per_row) * rd[0]), 2),
           round(graveyard[1] + square * ((k % per_row) * gd[1] - (k // per_row) * rd[1]), 2)] for k in range(48)]   # 30 captures + up to 16 promoting pawns
    spec = {"square_mm": square, "squares": centres, "graveyard_slots": gy,
            "made_by": "make_grid", "params": {"a1": a1, "file_dir_deg": file_dir_deg, "rank_dir_deg": rank_dir_deg,
                                               "graveyard": graveyard, "graveyard_dir_deg": graveyard_dir_deg}}
    if reserve_origin is not None:
        spec["reserve_origin"] = reserve_origin
    return spec


class Board:
    def __init__(self, spec):
        self.spec = spec
        self.square = float(spec["square_mm"])
        self.centre = {chess.parse_square(k): tuple(v) for k, v in spec["squares"].items()}
        if len(self.centre) != 64:
            raise ValueError(f"board spec has {len(self.centre)} squares, need 64")
        self.graveyard = [tuple(p) for p in spec["graveyard_slots"]]

    def nearest(self, xy):
        """(square, distance_mm) of the nearest square centre."""
        sq = min(self.centre, key=lambda s: math.dist(self.centre[s], xy))
        return sq, math.dist(self.centre[sq], xy)


# ---------------------------------------------------------------- pieces
def default_piece_map():
    """IDs 100-131: the 32 starting pieces; 132-139: promotion reserve (per colour: Q, Q, N, R)."""
    order = "RNBQKBNR" + "P" * 8
    m, tid = {}, 100
    for colour_upper in (True, False):
        for p in order:
            m[tid] = p if colour_upper else p.lower()
            tid += 1
    for s in ("Q", "Q", "N", "R", "q", "q", "n", "r"):
        m[tid] = s
        tid += 1
    return m


def load_pieces(path):
    d = json.load(open(path))
    return {int(k): v for k, v in d["tags"].items()}


def start_layout(pieces):
    """Physical start position: square -> tag id, from a piece map in its canonical order."""
    layout, board = {}, chess.Board()
    free = {}
    for tid in sorted(pieces):
        free.setdefault(pieces[tid], []).append(tid)
    for sq in sorted(board.piece_map()):
        free_list = free[board.piece_at(sq).symbol()]
        layout[sq] = free_list.pop(0)
    reserve = [t for lst in free.values() for t in lst]
    return layout, sorted(reserve)


# ---------------------------------------------------------------- physical state + steps
class Physical:
    """Where every tag is: on a square, in a graveyard slot, or in the reserve."""

    def __init__(self, board, pieces, layout, reserve):
        self.board, self.pieces = board, pieces
        self.on = dict(layout)                       # square -> tag
        self.grave = [None] * len(board.graveyard)   # slot -> tag
        self.reserve = list(reserve)                 # tags off the board, positions in board spec reserve area

    def reserve_xy(self, tid):
        """Reserve pieces stand in a row from reserve_origin along +y, one square apart, in tag order (IDs 132-139)."""
        o = self.board.spec.get("reserve_origin")
        if o is None or tid < 132:
            return None
        return (o[0], o[1] + (tid - 132) * self.board.square)

    def plan(self, pos, move):
        """Physical steps for `move` in chess position `pos` (before the move). Does not change state."""
        steps = []
        grave_free = [i for i, t in enumerate(self.grave) if t is None]

        def to_grave(sq, why):
            slot = grave_free.pop(0)
            steps.append({"kind": "move", "tag": self.on[sq], "piece": self.pieces[self.on[sq]], "why": why,
                          "from": chess.square_name(sq), "to": f"grave{slot}",
                          "from_xy": self.board.centre[sq], "to_xy": self.board.graveyard[slot]})

        mover = self.on[move.from_square]
        if pos.is_en_passant(move):
            to_grave(move.to_square + (-8 if pos.turn == chess.WHITE else 8), "en passant capture")
        elif pos.is_capture(move):
            to_grave(move.to_square, "capture")
        if move.promotion:
            to_grave(move.from_square, "promoting pawn")
            want = chess.Piece(move.promotion, pos.turn).symbol()
            src = next((t for t in self.reserve if self.pieces[t] == want), None)
            if src is not None:
                steps.append({"kind": "move", "tag": src, "piece": want, "why": "promotion (reserve)", "from": "reserve",
                              "to": chess.square_name(move.to_square), "from_xy": self.reserve_xy(src),
                              "to_xy": self.board.centre[move.to_square]})
            else:
                slot = next((i for i, t in enumerate(self.grave) if t is not None and self.pieces[t] == want), None)
                if slot is not None:
                    steps.append({"kind": "move", "tag": self.grave[slot], "piece": want, "why": "promotion (graveyard)",
                                  "from": f"grave{slot}", "to": chess.square_name(move.to_square),
                                  "from_xy": self.board.graveyard[slot], "to_xy": self.board.centre[move.to_square]})
                else:
                    steps.append({"kind": "human", "piece": want, "to": chess.square_name(move.to_square),
                                  "why": f"no spare {want}: ask the human to place one"})
            return steps
        steps.append({"kind": "move", "tag": mover, "piece": self.pieces[mover], "why": "move",
                      "from": chess.square_name(move.from_square), "to": chess.square_name(move.to_square),
                      "from_xy": self.board.centre[move.from_square], "to_xy": self.board.centre[move.to_square]})
        if pos.is_castling(move):
            rank = chess.square_rank(move.from_square)
            kingside = chess.square_file(move.to_square) == 6
            rf, rt = chess.square(7 if kingside else 0, rank), chess.square(5 if kingside else 3, rank)
            steps.append({"kind": "move", "tag": self.on[rf], "piece": self.pieces[self.on[rf]], "why": "castling rook",
                          "from": chess.square_name(rf), "to": chess.square_name(rt),
                          "from_xy": self.board.centre[rf], "to_xy": self.board.centre[rt]})
        return steps

    def apply(self, steps, human_tag_for=None):
        """Update the state as if the steps were executed. `human_tag_for(symbol)` returns a tag the human placed."""
        for s in steps:
            if s["kind"] == "human":
                tid = human_tag_for(s["piece"]) if human_tag_for else None
                if tid is None:
                    raise RuntimeError(f"human step not resolved: {s}")
                self.on[chess.parse_square(s["to"])] = tid
                continue
            tid, src, dst = s["tag"], s["from"], s["to"]
            if src == "reserve":
                self.reserve.remove(tid)
            elif src.startswith("grave"):
                self.grave[int(src[5:])] = None
            else:
                assert self.on.pop(chess.parse_square(src)) == tid
            if dst.startswith("grave"):
                self.grave[int(dst[5:])] = tid
            else:
                self.on[chess.parse_square(dst)] = tid

    def symbols(self):
        return {sq: self.pieces[t] for sq, t in self.on.items()}


def board_symbols(pos):
    return {sq: p.symbol() for sq, p in pos.piece_map().items()}


def detect_move(pos, observed):
    """Legal moves of `pos` whose result matches the observed square -> piece-symbol map (normally exactly one)."""
    hits = []
    for m in pos.legal_moves:
        pos.push(m)
        if board_symbols(pos) == observed:
            hits.append(m)
        pos.pop()
    return hits


# ---------------------------------------------------------------- observation (find_blocks.py JSON)
def observe(blocks_json, board, pieces):
    """square -> piece symbol from TOP-tag blocks; warnings for unknown tags, off-square pieces, duplicates."""
    d = json.load(open(blocks_json))
    obs, where, warn = {}, {}, []
    for b in d["blocks"]:
        if not b.get("top_face_tag"):
            warn.append(f"tag {b['id']}: not a top-face read, ignored")
            continue
        tid = int(b["id"])
        if tid not in pieces:
            warn.append(f"tag {tid}: not a chess piece")
            continue
        xy = b.get("plane_xy_mm") or b["block_center_mm"][:2]
        sq, dist = board.nearest(xy)
        if dist > SNAP_TOL * board.square:
            warn.append(f"tag {tid} ({pieces[tid]}) at {xy}: {dist:.0f} mm from {chess.square_name(sq)}, off-square")
            continue
        if sq in obs:
            warn.append(f"{chess.square_name(sq)}: two pieces (tags {where[sq]} and {tid})")
            continue
        obs[sq], where[sq] = pieces[tid], tid
    return obs, where, warn


def placement_fen(symbols):
    b = chess.Board(None)
    for sq, s in symbols.items():
        b.set_piece_at(sq, chess.Piece.from_symbol(s))
    return b.board_fen()


# ---------------------------------------------------------------- self-test (game logic only; test fixtures, no data)
OPERA_GAME = ("e4 e5 Nf3 d6 d4 Bg4 dxe5 Bxf3 Qxf3 dxe5 Bc4 Nf6 Qb3 Qe7 Nc3 c6 Bg5 b5 Nxb5 cxb5 Bxb5+ Nbd7 O-O-O Rd8 "
              "Rxd7 Rxd7 Rd1 Qe6 Bxd7+ Nxd7 Qb8+ Nxb8 Rd8#")   # Morphy vs Duke Karl / Count Isouard, Paris 1858


def run_game(board, pieces, sans_or_moves, pos=None, rng=None, max_plies=300):
    """Replay moves (SAN list) or random legal moves (rng) through plan/apply/detect; assert consistency every ply."""
    pos = pos or chess.Board()
    layout, reserve = start_layout(pieces)
    phys = Physical(board, pieces, layout, reserve)
    spare = iter(range(1000, 2000))   # tags handed over by the "human" in the test
    stats = {"plies": 0, "steps": 0, "captures": 0, "castles": 0, "ep": 0, "promotions": 0, "human": 0}

    def human(symbol):
        t = next(spare)
        pieces[t] = symbol
        stats["human"] += 1
        return t

    for ply in range(max_plies):
        if rng is None:
            if ply >= len(sans_or_moves):
                break
            move = pos.parse_san(sans_or_moves[ply])
        else:
            if pos.is_game_over():
                break
            legal = list(pos.legal_moves)
            promos = [m for m in legal if m.promotion]
            move = rng.choice(promos) if promos and rng.random() < 0.8 else rng.choice(legal)
        stats["captures"] += pos.is_capture(move)
        stats["castles"] += pos.is_castling(move)
        stats["ep"] += pos.is_en_passant(move)
        stats["promotions"] += bool(move.promotion)
        before = pos.copy()
        steps = phys.plan(pos, move)
        phys.apply(steps, human)
        pos.push(move)
        stats["plies"] += 1
        stats["steps"] += len(steps)
        assert phys.symbols() == board_symbols(pos), f"physical != logical after {move} ({pos.fen()})"
        found = detect_move(before, phys.symbols())
        assert found == [move], f"detect_move {found} != {move}"
        for s in steps:
            if s["kind"] == "move" and s["to"].startswith("grave"):
                assert s["to_xy"] is not None
    return pos, stats


def selftest():
    pieces = default_piece_map()
    board = Board(make_grid([150.0, -157.5], 45.0, 90.0, 0.0, [120.0, -157.5], 90.0, reserve_origin=[600.0, -157.5]))
    print("test board (geometry for the test only): a1", board.centre[chess.A1], "h8", board.centre[chess.H8])

    pos, st = run_game(board, pieces, OPERA_GAME.split())
    assert pos.is_checkmate(), "Opera game should end in mate"
    print("Opera game 1858:", st, "-> checkmate OK")

    for name, fen, sans in [
        ("en passant", "rnbqkbnr/ppp1p1pp/8/3pPp2/8/8/PPPP1PPP/RNBQKBNR w KQkq f6 0 3", ["exf6"]),
        ("castling both sides", "r3k2r/pppppppp/8/8/8/8/PPPPPPPP/R3K2R w KQkq - 0 1", ["O-O", "O-O-O"]),
        ("promotion with capture + underpromotion", "1n2k3/P6P/8/8/8/8/8/4K3 w - - 0 1", ["axb8=Q+", "Kd7", "h8=N"]),
    ]:
        # special positions: build the physical layout from the FEN with tags drawn from the piece map
        p = chess.Board(fen)
        free = {}
        for t in sorted(pieces):
            free.setdefault(pieces[t], []).append(t)
        layout = {sq: free[pc.symbol()].pop(0) for sq, pc in p.piece_map().items()}
        reserve = [t for t in sorted(pieces) if t >= 132 and t not in layout.values()]
        phys = Physical(board, pieces, layout, reserve)
        for san in sans:
            m = p.parse_san(san)
            before = p.copy()
            steps = phys.plan(p, m)
            phys.apply(steps)
            p.push(m)
            assert phys.symbols() == board_symbols(p), name
            assert detect_move(before, phys.symbols()) == [m], name
            print(f"  {name}: {san:8s} ->", "; ".join(f"{s['piece']} {s['from']}->{s['to']} ({s['why']})"
                                                     for s in steps if s["kind"] == "move"))

    # stress: many games of random legal moves (a test of the rules code, not data); promotions favoured
    rng = random.Random(0)
    tot = {}
    for g in range(300):
        _, st = run_game(board, dict(pieces), None, rng=rng)
        for k, v in st.items():
            tot[k] = tot.get(k, 0) + v
    print("300 random-move games:", tot)
    assert tot["ep"] > 0 and tot["castles"] > 0 and tot["promotions"] > 0 and tot["human"] > 0

    # observation: blocks at square centres (+ one off-square, one unknown tag, one side read) -> FEN
    layout, _ = start_layout(pieces)
    blocks = [{"id": t, "top_face_tag": True, "plane_xy_mm": [board.centre[sq][0] + 6, board.centre[sq][1] - 5]}
              for sq, t in layout.items()]
    blocks += [{"id": 3, "top_face_tag": True, "plane_xy_mm": [0, 0]},
               {"id": 132, "top_face_tag": True, "plane_xy_mm": [board.centre[chess.E4][0] + 20, board.centre[chess.E4][1]]},
               {"id": 140, "top_face_tag": False, "block_center_mm": [0, 0, 0]}]
    tmp = os.path.join(os.environ.get("TMPDIR", "/tmp"), f"chess_selftest_blocks_{os.getpid()}.json")
    json.dump({"blocks": blocks}, open(tmp, "w"))
    obs, _, warn = observe(tmp, board, pieces)
    os.remove(tmp)
    assert placement_fen(obs) == chess.Board().board_fen(), placement_fen(obs)
    assert len(warn) == 3, warn
    print("observe:", placement_fen(obs), "| warnings:", warn)
    print("SELFTEST PASSED")


# ---------------------------------------------------------------- terminal game vs Stockfish (no arm)
def play(board, pieces, robot_color, think_s, skill):
    layout, reserve = start_layout(pieces)
    phys, pos = Physical(board, pieces, layout, reserve), chess.Board()
    eng = chess.engine.SimpleEngine.popen_uci(STOCKFISH)
    eng.configure({"Skill Level": skill})
    extra = iter(range(3000, 4000))

    def human(symbol):
        """A piece the human put on the board (no spare of that type); tracked under a pseudo tag."""
        t = next(extra)
        pieces[t] = symbol
        return t

    try:
        while not pos.is_game_over():
            print("\n" + str(pos))
            if pos.turn == robot_color:
                move = eng.play(pos, chess.engine.Limit(time=think_s)).move
                print(f"robot plays {pos.san(move)}")
            else:
                txt = input("your move (SAN, 'quit'): ").strip()
                if txt == "quit":
                    break
                try:
                    move = pos.parse_san(txt)
                except ValueError as e:
                    print("  ", e)
                    continue
            steps = phys.plan(pos, move)
            for s in steps:
                if s["kind"] == "human":
                    print(f"   HUMAN: {s['why']}")
                else:
                    fx = "?" if s["from_xy"] is None else f"({s['from_xy'][0]:.1f}, {s['from_xy'][1]:.1f})"
                    print(f"   {'ARM' if pos.turn == robot_color else 'you'}: tag {s['tag']} {s['piece']} "
                          f"{s['from']} {fx} -> {s['to']} ({s['to_xy'][0]:.1f}, {s['to_xy'][1]:.1f})  [{s['why']}]")
            phys.apply(steps, human_tag_for=human)
            pos.push(move)
        print("\nresult:", pos.result(), pos.outcome())
    finally:
        eng.quit()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--pieces", default=DEFAULT_PIECES)
    ap.add_argument("--board")
    ap.add_argument("--make-grid", metavar="OUT.json")
    ap.add_argument("--a1", type=float, nargs=2)
    ap.add_argument("--square", type=float, default=45.0)
    ap.add_argument("--file-dir-deg", type=float, default=90.0)
    ap.add_argument("--rank-dir-deg", type=float, default=0.0)
    ap.add_argument("--graveyard", type=float, nargs=2)
    ap.add_argument("--graveyard-dir-deg", type=float, default=90.0)
    ap.add_argument("--reserve-origin", type=float, nargs=2)
    ap.add_argument("--observe", metavar="BLOCKS.json")
    ap.add_argument("--play", action="store_true")
    ap.add_argument("--robot-color", choices=["white", "black"], default="black")
    ap.add_argument("--think", type=float, default=0.3)
    ap.add_argument("--skill", type=int, default=5, help="Stockfish Skill Level 0-20")
    args = ap.parse_args()
    print(f"chess_core {datetime.datetime.now().isoformat(timespec='seconds')}  python {platform.python_version()}  "
          f"python-chess {chess.__version__}")

    if args.selftest:
        selftest()
        return 0
    if args.make_grid:
        if args.a1 is None or args.graveyard is None:
            ap.error("--make-grid needs --a1 and --graveyard")
        spec = make_grid(args.a1, args.square, args.file_dir_deg, args.rank_dir_deg, args.graveyard,
                         args.graveyard_dir_deg, args.reserve_origin)
        spec["created"] = datetime.datetime.now().isoformat(timespec="seconds")
        spec["created_by"] = os.path.basename(__file__)
        json.dump(spec, open(args.make_grid, "w"), indent=1)
        print("wrote", args.make_grid)
        return 0
    if not args.board:
        ap.error("--observe and --play need --board")
    board, pieces = Board(json.load(open(args.board))), load_pieces(args.pieces)
    if args.observe:
        obs, where, warn = observe(args.observe, board, pieces)
        print("placement FEN:", placement_fen(obs) if obs else "(no pieces on squares)")
        for w in warn:
            print("  warning:", w)
        return 0
    if args.play:
        play(board, pieces, chess.WHITE if args.robot_color == "white" else chess.BLACK, args.think, args.skill)
        return 0
    ap.print_help()
    return 1


if __name__ == "__main__":
    sys.exit(main())
