<script setup lang="ts"></script>

<template>
  <div>
    <LayoutHeader />
    <slot />
  </div>
</template>

<style scoped lang="scss">
body,
html {
  width: 100%;
  height: 100%;
  padding: 0;
  margin: 0;
}
</style>
