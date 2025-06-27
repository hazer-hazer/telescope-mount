<template>
  <main class="container">
    <Button variant="destructive" @click="server?.disconnect()">
      Disconnect
    </Button>
  </main>
</template>

<script setup lang="ts">
import { Button } from '@/components/ui/button';
import router from '@/router';
import { useBluetooth } from '@vueuse/core';
import { effect } from 'vue';

const {
  isConnected,
  server,
} = useBluetooth()

effect(() => {
  if (!isConnected.value) {
    console.log('disconeccted')
    router.push('/connect')
  } else {
    console.log('connected', server.value)
  }
})
</script>
