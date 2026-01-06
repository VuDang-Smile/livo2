/// <reference types="react-scripts" />

declare module 'three/examples/jsm/loaders/PCDLoader' {
  import { Loader, Points } from 'three';

  export class PCDLoader extends Loader {
    load(
      url: string,
      onLoad: (points: Points) => void,
      onProgress?: (event: ProgressEvent<EventTarget>) => void,
      onError?: (event: ErrorEvent) => void
    ): void;
  }
}