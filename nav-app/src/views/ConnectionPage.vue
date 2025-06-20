<template>
  <main>
    <div>
      <Button variant="secondary" @click="connect">Connect</Button>
    </div>
  </main>
</template>

<script setup lang="ts">
import { Button } from '@/components/ui/button'

const connect = () => {
  alert('kek')
}
</script>
