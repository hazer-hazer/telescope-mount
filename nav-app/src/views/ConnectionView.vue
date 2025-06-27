<template>
  <main>
    <div v-if="!isSupported">
      <Alert variant="destructive">
        <AlertCircle class="w-4 h-4" />
        <AlertTitle>Bluetooth not supported!</AlertTitle>
        <AlertDescription>
          Your session has expired. Please log in again.
        </AlertDescription>
        <AlertDescription>
          We're sorry, but your device/browser does not support bluetooth web API, please use another device or browser.
        </AlertDescription>
      </Alert>
    </div>
    <div v-else-if="!isConnected">
      <Button variant="secondary" @click="requestDevice">
        <BluetoothSearching class="w-4 h-4"></BluetoothSearching>
        Connect
      </Button>
    </div>

    <div v-if="isConnected && device">
      <TelemountControl :device="device"></TelemountControl>
    </div>
  </main>
</template>

<script setup lang="ts">
import { Button } from '@/components/ui/button';
import { Alert, AlertTitle, AlertDescription } from '@/components/ui/alert';
import { AlertCircle } from 'lucide-vue-next';
import { useBluetooth } from '@vueuse/core';
import { BluetoothSearching } from 'lucide-vue-next';
import TelemountControl from '@/components/TelemountControl.vue';

const {
  isConnected,
  isSupported,
  requestDevice,
  device,
} = useBluetooth({
  acceptAllDevices: true,
})

</script>
