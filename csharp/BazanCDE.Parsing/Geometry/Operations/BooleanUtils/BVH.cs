using System;
using System.Collections.Generic;

namespace BazanCDE.Parsing.Geometry.Operations.BooleanUtils
{
    public class BVHNode
    {
        public uint Start;
        public uint End;
        public uint Left = 0;
        public uint Right = 0;
        public AABB Box = new();

        public bool IsLeaf()
        {
            return Left == 0 && Right == 0;
        }

        public bool Overlaps(BVHNode other)
        {
            return Box.Intersects(other.Box);
        }
    }
    
    public class BVH
    {
        public AABB Box = new();
        public List<AABB> Boxes = new();
        public List<BVHNode> Nodes = new();
        public Geometry? Ptr;

        public void Intersect(BVH other, Action<uint, uint> callback)
        {
            if (Nodes.Count == 0 || other.Nodes.Count == 0)
            {
                return;
            }

            var bvhStack = new Stack<(uint I1, uint I2)>();
            bvhStack.Push((0, 0));

            while (bvhStack.Count > 0)
            {
                var (i1, i2) = bvhStack.Pop();

                var n1 = Nodes[(int)i1];
                var n2 = other.Nodes[(int)i2];

                if (!n1.Overlaps(n2))
                {
                    continue;
                }

                if (n1.IsLeaf() && n2.IsLeaf())
                {
                    // both leaves, compare contents
                    for (var i = n1.Start; i < n1.End; i++)
                    {
                        for (var j = n2.Start; j < n2.End; j++)
                        {
                            if (Boxes[(int)i].Intersects(other.Boxes[(int)j]))
                            {
                                callback(Boxes[(int)i].Index, other.Boxes[(int)j].Index);
                            }
                        }
                    }
                }
                else if (n1.IsLeaf())
                {
                    bvhStack.Push((i1, n2.Left));
                    bvhStack.Push((i1, n2.Right));
                }
                else if (n2.IsLeaf())
                {
                    bvhStack.Push((n1.Left, i2));
                    bvhStack.Push((n1.Right, i2));
                }
                else
                {
                    // neither are leaves, split n1
                    bvhStack.Push((n1.Left, i2));
                    bvhStack.Push((n1.Right, i2));
                }
            }
        }

        public bool IntersectRay(Vec3 origin, Vec3 dir, Func<uint, bool> callback)
        {
            if (Nodes.Count == 0)
            {
                return false;
            }

            var stack = new List<uint>(capacity: 64) { 0 };
            while (stack.Count > 0)
            {
                var last = stack.Count - 1;
                var nodeIndex = stack[last];
                stack.RemoveAt(last);

                var node = Nodes[(int)nodeIndex];
                if (!node.Box.Intersect(origin, dir))
                {
                    continue;
                }

                // hit
                if (node.IsLeaf())
                {
                    // check boxes
                    for (var i = node.Start; i < node.End; i++)
                    {
                        var box = Boxes[(int)i];
                        if (!box.Intersect(origin, dir))
                        {
                            continue;
                        }

                        if (callback(box.Index))
                        {
                            return true;
                        }
                    }
                }
                else
                {
                    // visit children
                    stack.Add(node.Left);
                    stack.Add(node.Right);
                }
            }

            return false;
        }

        public static int MakeBVH(
            List<AABB> boxes,
            List<BVHNode> nodes,
            int start,
            int end,
            int axis,
            int depth,
            ref int offset)
        {
            var nodeId = offset++;
            while (nodes.Count <= nodeId)
            {
                nodes.Add(new BVHNode());
            }

            var node = nodes[nodeId];
            node.Start = (uint)start;
            node.End = (uint)end;
            node.Left = 0;
            node.Right = 0;
            node.Box = new AABB();

            for (var i = start; i < end; i++)
            {
                node.Box.Merge(boxes[i]);
            }

            if (depth == 6)
            {
                return nodeId;
            }

            var size = end - start;
            // ignore cubes
            if (size <= 12)
            {
                return nodeId;
            }

            var middle = (end + start) / 2;

            // C++ uses nth_element; segment sort keeps the same split criteria.
            boxes.Sort(
                start,
                size,
                Comparer<AABB>.Create((first, second) =>
                    GetAxis(second.Center, axis).CompareTo(GetAxis(first.Center, axis))));

            nodes[nodeId].Left = (uint)MakeBVH(boxes, nodes, start, middle, (axis + 1) % 3, depth + 1, ref offset);
            nodes[nodeId].Right = (uint)MakeBVH(boxes, nodes, middle, end, (axis + 1) % 3, depth + 1, ref offset);

            return nodeId;
        }

        public static BVH MakeBVH(Geometry mesh)
        {
            var bvh = new BVH
            {
                Ptr = mesh,
                Boxes = new List<AABB>((int)mesh.NumFaces)
            };

            for (var i = 0u; i < mesh.NumFaces; i++)
            {
                bvh.Boxes.Add(mesh.GetFaceBox((int)i));
                bvh.Box.Merge(bvh.Boxes[(int)i]);
            }

            var offset = 0;
            // start BVH at Y axis
            MakeBVH(bvh.Boxes, bvh.Nodes, 0, bvh.Boxes.Count, 1, 0, ref offset);

            return bvh;
        }

        private static double GetAxis(Vec3 value, int axis)
        {
            return axis switch
            {
                0 => value.X,
                1 => value.Y,
                _ => value.Z
            };
        }
    }
}
