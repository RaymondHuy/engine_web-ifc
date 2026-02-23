using EngineGeometry = BazanCDE.Parsing.Geometry.Operations.BooleanUtils.Geometry;
using EngineSimplePlane = BazanCDE.Parsing.Geometry.Operations.BooleanUtils.SimplePlane;

namespace BazanCDE.Parsing.Geometry.Operations.BimGeometry
{
    public static partial class Utils
    {
        public static EngineGeometry convertToEngine(Geometry geom)
        {
            var newGeom = new EngineGeometry();
            newGeom.FVertexData = geom.fvertexData;
            newGeom.VertexData = geom.vertexData;
            newGeom.IndexData = geom.indexData;
            newGeom.PlaneData = geom.planeData;
            newGeom.NumPoints = geom.numPoints;
            newGeom.NumFaces = geom.numFaces;

            foreach (var plane in geom.planes)
            {
                var newPlane = new EngineSimplePlane(plane.normal, plane.distance);
                newGeom.Planes.Add(newPlane);
            }

            newGeom.HasPlanes = geom.hasPlanes;
            return newGeom;
        }

        public static Geometry convertToBim(EngineGeometry geom)
        {
            var newGeom = new Geometry
            {
                hasPlanes = geom.HasPlanes,
                numPoints = geom.NumPoints,
                numFaces = geom.NumFaces
            };

            newGeom.fvertexData.AddRange(geom.FVertexData);
            newGeom.vertexData.AddRange(geom.VertexData);
            newGeom.indexData.AddRange(geom.IndexData);
            newGeom.planeData.AddRange(geom.PlaneData);

            uint id = 0;
            foreach (var plane in geom.Planes)
            {
                var newPlane = new Plane
                {
                    id = id,
                    distance = plane.Distance,
                    normal = plane.Normal
                };
                newGeom.planes.Add(newPlane);
                id++;
            }

            return newGeom;
        }

        public static Geometry Union(Geometry firstOperator, Geometry secondOperator)
        {
            var firstEngGeom = convertToEngine(firstOperator);
            var secondEngGeom = convertToEngine(secondOperator);
            return convertToBim(BazanCDE.Parsing.Geometry.Operations.BooleanUtils.Utils.Union(firstEngGeom, secondEngGeom));
        }

        public static Geometry Subtract(Geometry firstOperator, Geometry secondOperator)
        {
            var firstEngGeom = convertToEngine(firstOperator);
            var secondEngGeom = convertToEngine(secondOperator);
            return convertToBim(BazanCDE.Parsing.Geometry.Operations.BooleanUtils.Utils.Subtract(firstEngGeom, secondEngGeom));
        }

        public static Geometry BoolProcess(Geometry firstOperator, List<Geometry> secondGeoms, string op)
        {
            var finalResult = new Geometry();

            foreach (var secondGeom in secondGeoms)
            {
                var doit = true;
                if (secondGeom.numFaces == 0)
                {
                    // Bail out because we can get strange meshes if parsing failed upstream.
                    doit = false;
                }

                if (firstOperator.numFaces == 0 && op != "UNION")
                {
                    // Bail out because we can get strange meshes if parsing failed upstream.
                    break;
                }

                if (doit)
                {
                    var secondOperator = secondGeom;
                    firstOperator.buildPlanes();
                    secondOperator.buildPlanes();

                    if (op == "DIFFERENCE")
                    {
                        firstOperator = Subtract(firstOperator, secondOperator);
                    }
                    else if (op == "UNION")
                    {
                        firstOperator = Union(firstOperator, secondOperator);
                    }
                }
            }

            finalResult.AddGeometry(firstOperator);
            return finalResult;
        }
    }
}
