import React from 'react';
import { ModelViewerElement } from '@google/model-viewer';

declare global {
  namespace JSX {
    interface IntrinsicElements {
      'model-viewer': React.DetailedHTMLProps<
        React.HTMLAttributes<ModelViewerElement>,
        ModelViewerElement
      >;
    }
  }
}

interface ModelViewerProps {
  src: string;
  alt: string;
  autoRotate?: boolean;
  cameraControls?: boolean;
  ar?: boolean;
  poster?: string;
  shadowIntensity?: string;
  environmentImage?: string;
}

export default function ModelViewer({
  src,
  alt,
  autoRotate = true,
  cameraControls = true,
  ar = false,
  poster,
  shadowIntensity = '1',
  environmentImage = 'neutral',
}: ModelViewerProps): JSX.Element {
  return (
    <model-viewer
      src={src}
      alt={alt}
      ar={ar}
      auto-rotate={autoRotate}
      camera-controls={cameraControls}
      poster={poster}
      shadow-intensity={shadowIntensity}
      environment-image={environmentImage}
      style={{ width: '100%', height: '400px' }}
    >
    </model-viewer>
  );
}
