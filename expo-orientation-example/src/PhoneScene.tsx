import { useMemo } from 'react';
import Svg, {
  Circle,
  G,
  Line,
  Polygon,
  Text as SvgText,
} from 'react-native-svg';
import type { FrameOrientation } from 'react-native-cumquat';

import { modelOrientation, rotateVector, type Vec3 } from './orientationMath';

type Props = {
  orientation: FrameOrientation | null;
  width: number;
  height: number;
};

type Projected = { x: number; y: number; depth: number };

const PHONE_VERTICES: readonly Vec3[] = [
  [-0.5, -0.94, -0.07],
  [0.5, -0.94, -0.07],
  [0.5, 0.94, -0.07],
  [-0.5, 0.94, -0.07],
  [-0.5, -0.94, 0.07],
  [0.5, -0.94, 0.07],
  [0.5, 0.94, 0.07],
  [-0.5, 0.94, 0.07],
];

const FACES = [
  { indices: [0, 1, 2, 3], color: '#16213a' },
  { indices: [0, 4, 5, 1], color: '#37445d' },
  { indices: [1, 5, 6, 2], color: '#2b374f' },
  { indices: [2, 6, 7, 3], color: '#45516a' },
  { indices: [3, 7, 4, 0], color: '#303d55' },
  { indices: [4, 7, 6, 5], color: '#07111f' },
] as const;

function toView([east, north, up]: Vec3): Vec3 {
  const yaw = (-35 * Math.PI) / 180;
  const pitch = (62 * Math.PI) / 180;
  const x1 = Math.cos(yaw) * east - Math.sin(yaw) * north;
  const y1 = Math.sin(yaw) * east + Math.cos(yaw) * north;
  return [
    x1,
    Math.cos(pitch) * y1 - Math.sin(pitch) * up,
    Math.sin(pitch) * y1 + Math.cos(pitch) * up,
  ];
}

function project(point: Vec3, width: number, height: number): Projected {
  const [x, y, depth] = toView(point);
  const cameraDistance = 5;
  const perspective = cameraDistance / (cameraDistance - depth);
  const scale = Math.min(width, height) * 0.27;
  return {
    x: width / 2 + x * scale * perspective,
    y: height / 2 - y * scale * perspective,
    depth,
  };
}

function points(vertices: readonly Projected[]) {
  return vertices.map(({ x, y }) => `${x},${y}`).join(' ');
}

export function PhoneScene({ orientation, width, height }: Props) {
  const scene = useMemo(() => {
    const q = orientation
      ? modelOrientation(orientation)
      : { x: 0, y: 0, z: 0, w: 1 };
    const rotated = PHONE_VERTICES.map((vertex) => rotateVector(vertex, q));
    const projected = rotated.map((vertex) => project(vertex, width, height));
    const faces = FACES.map((face) => ({
      ...face,
      vertices: face.indices.map((index) => projected[index]!),
      depth:
        face.indices.reduce<number>(
          (sum, index) => sum + projected[index]!.depth,
          0
        ) / face.indices.length,
    })).sort((left, right) => left.depth - right.depth);

    const deviceAxes = [
      { label: '+X', color: '#ff6577', vector: [1.35, 0, 0] as Vec3 },
      { label: '+Y', color: '#71e6a3', vector: [0, 1.35, 0] as Vec3 },
      { label: '+Z', color: '#60c9ff', vector: [0, 0, 1.35] as Vec3 },
    ].map((axis) => ({
      ...axis,
      end: project(rotateVector(axis.vector, q), width, height),
    }));

    const worldAxes = [
      { label: 'E', color: '#7e4050', vector: [1.55, 0, 0] as Vec3 },
      { label: 'N', color: '#3e7657', vector: [0, 1.55, 0] as Vec3 },
      { label: 'U', color: '#376881', vector: [0, 0, 1.55] as Vec3 },
    ].map((axis) => ({ ...axis, end: project(axis.vector, width, height) }));

    ///////////
    // const bearing = ((Math.atan2(forward.x, forward.y) * 180) / Math.PI + 360) % 360;
    ///////////

    return {
      faces,
      deviceAxes,
      worldAxes,
      origin: project([0, 0, 0], width, height),
    };
  }, [height, orientation, width]);

  return (
    <Svg width={width} height={height} viewBox={`0 0 ${width} ${height}`}>
      <G opacity={0.55}>
        {scene.worldAxes.map((axis) => (
          <G key={axis.label}>
            <Line
              x1={scene.origin.x}
              y1={scene.origin.y}
              x2={axis.end.x}
              y2={axis.end.y}
              stroke={axis.color}
              strokeWidth={1.5}
              strokeDasharray="5 6"
            />
            <SvgText
              x={axis.end.x}
              y={axis.end.y}
              fill={axis.color}
              fontSize={12}
            >
              {axis.label}
            </SvgText>
          </G>
        ))}
      </G>

      {scene.faces.map((face, index) => (
        <Polygon
          key={`${face.indices.join('-')}-${index}`}
          points={points(face.vertices)}
          fill={face.color}
          stroke="#91a4c4"
          strokeWidth={1.2}
          strokeLinejoin="round"
        />
      ))}

      <Circle cx={scene.origin.x} cy={scene.origin.y} r={4} fill="#f4f7ff" />
      {scene.deviceAxes.map((axis) => (
        <G key={axis.label}>
          <Line
            x1={scene.origin.x}
            y1={scene.origin.y}
            x2={axis.end.x}
            y2={axis.end.y}
            stroke={axis.color}
            strokeWidth={3}
            strokeLinecap="round"
          />
          <Circle cx={axis.end.x} cy={axis.end.y} r={4} fill={axis.color} />
          <SvgText
            x={axis.end.x + 6}
            y={axis.end.y - 6}
            fill={axis.color}
            fontSize={13}
            fontWeight="700"
          >
            {axis.label}
          </SvgText>
        </G>
      ))}
    </Svg>
  );
}
