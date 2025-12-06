import React, { useRef, useEffect, useState, Suspense } from 'react';
import { Canvas, useLoader } from '@react-three/fiber';
import { OrbitControls } from '@react-three/drei';
import { URDFLoader } from 'urdf-loader';
import { LoadingManager } from 'three';

interface URDFViewerProps {
  urdf: string; // Path to the URDF file
  scale?: number;
  position?: [number, number, number];
  rotation?: [number, number, number];
}

const UrdfModel: React.FC<{ urdf: string }> = ({ urdf }) => {
  const [model, setModel] = useState<any>(null); // TODO: Type this properly
  const manager = useRef(new LoadingManager());

  useEffect(() => {
    const loader = new URDFLoader(manager.current);
    loader.load(urdf, (result) => {
      setModel(result);
    });
  }, [urdf]);

  if (!model) {
    return null;
  }

  return <primitive object={model} />;
};

const URDFViewer: React.FC<URDFViewerProps> = ({
  urdf,
  scale = 1,
  position = [0, 0, 0],
  rotation = [0, 0, 0],
}) => {
  return (
    <div style={{ height: '500px', width: '100%', background: '#222' }}>
      <Canvas camera={{ position: [2, 2, 2] }}>
        <ambientLight intensity={0.5} />
        <spotLight position={[10, 10, 10]} angle={0.15} penumbra={1} />
        <pointLight position={[-10, -10, -10]} />
        <Suspense fallback={null}>
          <UrdfModel urdf={urdf} />
        </Suspense>
        <OrbitControls />
      </Canvas>
    </div>
  );
};

export default URDFViewer;
