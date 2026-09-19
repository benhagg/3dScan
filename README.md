# Quickstart Commands

## Expo Login
```bash
npx expo login
```

## Real-Time Live Visualizer
Terminal 1 (Server):
```bash
npm run server
```

Terminal 2 (Visualizer):
```bash
npm run live
```

Terminal 3 (App):
```bash
npx expo start
```
*(On iPhone: tap "Start Realtime Live Stream")*

## View Recorded Session (Offline)
```bash
npm run view
```

## Native Dev Build (LiDAR Support)
```bash
npx expo prebuild -p ios
npx expo run:ios --device
```

