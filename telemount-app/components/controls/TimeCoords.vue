<template>
  <div>
    
  </div>
</template>

<script lang="ts" setup>
const {secs} = defineProps<{
  secs: number
}>()

const hours = computed(() => secs / 3600.0)
const minutes = computed(() => secs / 60.0)
</script>

<style>

</style>