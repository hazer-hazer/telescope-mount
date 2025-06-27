import { defineStore } from 'pinia'

export const useMyBtStore = defineStore('telemount-device', {
  state: (): {
    name: string | null
    id: string | null
  } => ({ name: null, id: null }),
  actions: {
    forget() {
      this.$reset()
    },
    connected(id: string, name: string) {
      this.id = id
      this.name = name
    }
  },
  persist: true,
})
